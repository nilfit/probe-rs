//! Sequences for NXP chips that use ARMv7-M cores.

use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};

use crate::{
    architecture::arm::{
        ap::{memory_ap::MemoryApType, AccessPortError, AccessPortType},
        armv7m::{FpCtrl, FpRev2CompX},
        communication_interface::Initialized,
        core::{
            armv7m::{Aircr, Dhcsr},
            registers::cortex_m::PC,
        },
        dp::{Abort, Ctrl, DpAccess as _, Select},
        memory::{adi_v5_memory_interface::ADIMemoryInterface, ArmMemoryInterface},
        sequences::{ArmDebugSequence, ArmDebugSequenceError},
        ArmCommunicationInterface, ArmError, DpAddress, FullyQualifiedApAddress,
    },
    core::MemoryMappedRegister,
    MemoryInterface as _,
};

/// Debug sequences for MIMXRT10xx MCUs.
///
/// In its current form, it uses no custom debug sequences. Instead, it ensures a reliable
/// reset sequence.
///
/// # On custom reset catch
///
/// Some tools use a custom reset catch that looks at the program image, finds the
/// reset vector, then places a breakpoint on that reset vector. This implementation
/// isn't doing that. That would be necessary if we don't control the kind of reset
/// that's happening. Since we're definitely using a SYSRESETREQ, we can rely on the
/// normal reset catch.
///
/// If the design changes such that the kind of reset isn't in our control, we'll
/// need to handle those cases.
#[derive(Debug)]
pub struct MIMXRT10xx(());

impl MIMXRT10xx {
    /// Create a sequence handle for the MIMXRT10xx.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self(()))
    }

    /// Runtime validation of core type.
    fn check_core_type(&self, core_type: crate::CoreType) -> Result<(), ArmError> {
        const EXPECTED: crate::CoreType = crate::CoreType::Armv7em;
        if core_type != EXPECTED {
            tracing::warn!(
                "MIMXRT10xx core type supplied as {core_type:?}, but the actual core is a {EXPECTED:?}"
            );
            // Not an issue right now. Warning because it's curious.
        }
        Ok(())
    }

    /// Use the boot fuse configuration for FlexRAM.
    ///
    /// If the user changed the FlexRAM configuration in software, this will undo
    /// that configuration, preferring the system's POR FlexRAM state.
    fn use_boot_fuses_for_flexram(
        &self,
        probe: &mut dyn ArmMemoryInterface,
    ) -> Result<(), ArmError> {
        const IOMUXC_GPR_GPR16: u64 = 0x400A_C040;
        const FLEXRAM_BANK_CFG_SEL_MASK: u32 = 1 << 2;
        let mut gpr16 = probe.read_word_32(IOMUXC_GPR_GPR16)?;
        gpr16 &= !FLEXRAM_BANK_CFG_SEL_MASK;
        probe.write_word_32(IOMUXC_GPR_GPR16, gpr16)?;
        probe.flush()?;
        Ok(())
    }
}

impl ArmDebugSequence for MIMXRT10xx {
    fn reset_system(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        core_type: crate::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.check_core_type(core_type)?;

        // OK to perform before the reset, since the configuration
        // persists beyond the reset.
        self.use_boot_fuses_for_flexram(interface)?;

        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);

        // Reset happens very quickly, and takes a bit. Ignore write and flush
        // errors that will occur due to the reset reaction.
        interface
            .write_word_32(Aircr::get_mmio_address(), aircr.into())
            .ok();
        interface.flush().ok();

        // Wait for the reset to finish...
        thread::sleep(Duration::from_millis(100));

        let start = Instant::now();
        loop {
            let dhcsr = match interface.read_word_32(Dhcsr::get_mmio_address()) {
                Ok(val) => Dhcsr(val),
                Err(ArmError::AccessPort {
                    source:
                        AccessPortError::RegisterRead { .. } | AccessPortError::RegisterWrite { .. },
                    ..
                }) => {
                    // Some combinations of debug probe and target (in
                    // particular, hs-probe and ATSAMD21) result in
                    // register read errors while the target is
                    // resetting.
                    //
                    // See here for more info: https://github.com/probe-rs/probe-rs/pull/1174#issuecomment-1275568493
                    continue;
                }
                Err(err) => return Err(err),
            };
            if !dhcsr.s_reset_st() {
                return Ok(());
            }

            if start.elapsed() >= Duration::from_millis(500) {
                return Err(ArmError::Timeout);
            }
        }
    }
}

/// Debug sequences for MIMXRT117x MCUs.
///
/// Currently only supports the Cortex M7. In fact, if you try to interact with the Cortex M4,
/// you'll have a bad time: its access port doesn't appear until it's released from reset!
/// For the time being, you can only do things through the CM7.
#[derive(Debug)]
pub struct MIMXRT117x {
    /// Given the reset we're performing, we won't be able to perform
    /// a normal vector catch. (The boot ROM doesn't care about us.)
    /// We'll simulate that behavior for the user.
    simulate_reset_catch: AtomicBool,
}

impl MIMXRT117x {
    /// System reset controller base address.
    const SRC: u64 = 0x40C0_4000;
    /// SRC reset mode register.
    const SRC_SRMR: u64 = Self::SRC + 4;

    fn new() -> Self {
        Self {
            simulate_reset_catch: AtomicBool::new(false),
        }
    }

    /// Create a sequence handle for the MIMXRT117x.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self::new())
    }

    /// To ensure we affect a system reset, clear the mask that would prevent
    /// a response to the CM7's SYSRESETREQ.
    fn clear_src_srmr_mask(&self, probe: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
        let mut srmr = probe.read_word_32(Self::SRC_SRMR)?;
        tracing::debug!("SRC_SRMR: {srmr:#010X}. Clearing the M7REQ_RESET_MODE mask...");
        srmr &= !(0b11 << 12);
        probe.write_word_32(Self::SRC_SRMR, srmr)?;
        probe.flush()?;
        Ok(())
    }

    /// Halt or unhalt the core.
    fn halt(&self, probe: &mut dyn ArmMemoryInterface, halt: bool) -> Result<(), ArmError> {
        let mut dhcsr = Dhcsr(probe.read_word_32(Dhcsr::get_mmio_address())?);
        dhcsr.set_c_halt(halt);
        dhcsr.set_c_debugen(true);
        dhcsr.enable_write();

        probe.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;
        probe.flush()?;

        let start = Instant::now();
        let action = if halt { "halt" } else { "unhalt" };

        while Dhcsr(probe.read_word_32(Dhcsr::get_mmio_address())?).s_halt() != halt {
            if start.elapsed() > Duration::from_millis(100) {
                tracing::debug!("Exceeded timeout while waiting for the core to {action}");
                return Err(ArmError::Timeout);
            }
            thread::sleep(Duration::from_millis(1));
        }

        Ok(())
    }

    /// Poll the AP's status until it can accept transfers.
    fn wait_for_enable(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        timeout: Duration,
    ) -> Result<(), ArmError> {
        let start = Instant::now();
        let mut errors = 0usize;
        let mut disables = 0usize;

        let (interface, memory_ap) = probe.try_as_parts()?;
        loop {
            match memory_ap.generic_status(interface) {
                Ok(csw) if csw.DeviceEn => {
                    tracing::debug!("Device enabled after {}ms with {errors} errors and {disables} invalid statuses", start.elapsed().as_millis());
                    return Ok(());
                }
                Ok(_) => disables += 1,
                Err(_) => errors += 1,
            }

            if start.elapsed() > timeout {
                tracing::debug!("Exceeded {}ms timeout while waiting for enable with {errors} errors and {disables} invalid statuses", timeout.as_millis());
                return Err(ArmError::Timeout);
            }

            thread::sleep(Duration::from_millis(1));
        }
    }

    /// Assumes that the core is halted.
    fn read_core_reg(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        reg: crate::core::registers::CoreRegister,
    ) -> Result<u32, ArmError> {
        crate::architecture::arm::core::cortex_m::read_core_reg(probe, reg.into())
    }

    /// Assumes that the core is halted.
    fn write_core_reg(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        reg: crate::core::registers::CoreRegister,
        value: u32,
    ) -> Result<(), ArmError> {
        crate::architecture::arm::core::cortex_m::write_core_reg(probe, reg.into(), value)?;
        probe.flush()?;
        Ok(())
    }

    /// Ensure that the program counter's contents match `expected`.
    ///
    /// Assumes that the core is halted.
    fn check_pc(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        expected: u32,
    ) -> Result<(), ArmDebugSequenceError> {
        let pc = self
            .read_core_reg(probe, PC)
            .map_err(|err| ArmDebugSequenceError::SequenceSpecific(err.into()))?;
        if pc != expected {
            let err = format!("The MIMXRT1170's Cortex M7 should be at address {expected:#010X} but it's at {pc:#010X}");
            return Err(ArmDebugSequenceError::SequenceSpecific(err.into()));
        }
        Ok(())
    }

    /// When the boot ROM detects a reset due to SYSRESETREQ, it spins
    /// at this location. It appears that this spinning location is after
    /// the boot ROM has done its useful work (like turn on clocks, prepare
    /// FlexSPI configuration blocks), but before it jumps into the program.
    const BOOT_ROM_SPIN_ADDRESS: u32 = 0x00223104;

    /// Returns the reset handler address contained in the NVM program image.
    ///
    /// We might not find that reset handler. In that case, return `None`.
    fn find_flexspi_image_reset_handler(
        &self,
        probe: &mut dyn ArmMemoryInterface,
    ) -> Result<Option<u32>, ArmError> {
        /// Assumed by today's in-tree target definition.
        const FLEXSPI1: u64 = 0x30000000;
        /// A well-formed FlexSPI program has its image vector table at this offset in flash.
        const IVT: u64 = FLEXSPI1 + 0x1000;
        tracing::debug!("Assuming that your CM7's program is in FlexSPI1 at {FLEXSPI1:#010X}");

        // Make sure the IVT header looks reasonable.
        //
        // See 10.7.1.1 Image vector table structure in the 1170 RM (Rev 2).
        // If it doesn't look reasonable, we assume that FlexSPI is inaccessible.
        let ivt_header = probe.read_word_32(IVT)?;
        tracing::debug!("IVT Header: {ivt_header:#010X}");

        if ivt_header & 0xFF != 0xD1 {
            tracing::debug!("IVT tag is incorrect! Expected 0xD1 in {ivt_header:#010X}");
            return Ok(None);
        }

        if (ivt_header >> 8) & 0xFFFF != 0x2000 {
            tracing::debug!("IVT length is incorrect! {ivt_header:#010X}");
            return Ok(None);
        }

        let ivt_version = ivt_header >> 24;
        if !(0x40..=0x45).contains(&ivt_version) {
            tracing::debug!("IVT version is invalid! {ivt_header:#010X}");
            return Ok(None);
        }

        // IVT versions 4.0 (0x40) are documented as containing the "entry point."
        // But in practice, this seems to be the pointer to the vector table. IVT
        // versions 4.1 and 4.3 (0x41, 0x43) appear to truly use the reset handler, not
        // the vector table. I can't find any documentation on this, so this comes from
        // some local testing. We assume that 4.0 is the outlier, and that all versions
        // above it use the same approach.
        let reset_handler = if ivt_version == 0x40 {
            // The address of the vector table is immediately behind the IVT header.
            let vector_table = probe.read_word_32(IVT + 4)?;
            tracing::debug!("Vector table address: {vector_table:#010X}");

            // The vector table starts with the stack pointer. Then the
            // reset handle is immediately behind that.
            probe.read_word_32(u64::from(vector_table) + 4u64)?
        } else {
            // The reset handler immediately follows the IVT header.
            probe.read_word_32(IVT + 4)?
        };

        tracing::debug!("Reset handler: {reset_handler:#010X}");
        if reset_handler & 1 == 0 {
            tracing::debug!(
                "Is your reset handler actually a function address? Where's its thumb bit?"
            );
            return Ok(None);
        }

        Ok(Some(reset_handler))
    }

    /// See documentation for [`MIMXRT10xx::use_boot_fuses_for_flexram`].
    fn use_boot_fuses_for_flexram(
        &self,
        probe: &mut dyn ArmMemoryInterface,
    ) -> Result<(), ArmError> {
        const IOMUXC_GPR_GPR16: u64 = 0x400E_4040;
        const FLEXRAM_BANK_CFG_SEL_MASK: u32 = 1 << 2;
        let mut gpr16 = probe.read_word_32(IOMUXC_GPR_GPR16)?;
        gpr16 &= !FLEXRAM_BANK_CFG_SEL_MASK;
        probe.write_word_32(IOMUXC_GPR_GPR16, gpr16)?;
        probe.flush()?;
        Ok(())
    }
}

impl ArmDebugSequence for MIMXRT117x {
    fn reset_catch_set(
        &self,
        _: &mut dyn ArmMemoryInterface,
        _: probe_rs_target::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.simulate_reset_catch.store(true, Ordering::Relaxed);
        Ok(())
    }
    fn reset_catch_clear(
        &self,
        _: &mut dyn ArmMemoryInterface,
        _: probe_rs_target::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.simulate_reset_catch.store(false, Ordering::Relaxed);
        Ok(())
    }

    fn reset_system(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        core_type: probe_rs_target::CoreType,
        debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        // OK to perform before the reset, since the configuration
        // persists beyond the reset.
        self.use_boot_fuses_for_flexram(probe)?;

        // Cache debug system state that may be lost across the reset.
        let debug_cache = DebugCache::from_target(probe)?;

        // Make sure that the CM7's SYSRESETREQ isn't ignored by the system
        // reset controller.
        self.clear_src_srmr_mask(probe)?;

        // Affect a SYSRESETREQ throught the CM7 to reset the entire system.
        //
        // For more information on the SYSRESETREQ response, consult the system
        // reset controller (SRC) section of the reference manual. This is a
        // convenient way to perform a whole-system reset.
        //
        // Another approach to perform this reset: iterate through all SRC slice controls,
        // and manually reset them. That should be close to SYSRESETREQ. However, it seems
        // that there are no slice controls for CM4MEM (LMEM) and CM7MEM (FlexRAM)
        // so you might not be able to affect a reset on those two domains.
        //
        // If you scan through the slices, you'll notice that the M7CORE and M7DEBUG are
        // different slices. You'll think "I can perform a reset through the SRC that hits
        // all slices except the M*DEBUG slices. This would preserve debugging and I won't
        // have to re-initialize the debug port!" I could not get that to work; if I did a
        // reset through SRC_CTRL_M7CORE, I found that I still needed to re-initialize the
        // debug port after the reset. Maybe I did something wrong.
        //
        // We're about to lose the debug port! We're ignoring missed or incorrect responses.
        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);
        probe
            .write_word_32(Aircr::get_mmio_address(), aircr.into())
            .ok();
        probe.flush().ok();

        // If all goes well, we lost the debug port. Thanks, boot ROM. Let's bring it back.
        //
        // The ARM communication interface knows how to re-initialize the debug port.
        // Re-initializing the core(s) is on us.
        let ap = probe.ap().ap_address().clone();
        let interface = probe.get_arm_communication_interface()?;
        interface.reinitialize()?;

        assert!(debug_base.is_none());
        self.debug_core_start(interface, &ap, core_type, None, None)?;

        // Are we back?
        self.wait_for_enable(probe, Duration::from_millis(300))?;

        // We're back. Halt the core so we can establish the reset context.
        self.halt(probe, true)?;

        // When we reset into the boot ROM, it checks why we reset. If the boot ROM observes that
        // we reset due to SYSRESETREQ, it spins at a known address. Are we spinning there?
        self.check_pc(probe, Self::BOOT_ROM_SPIN_ADDRESS)?;

        // Why does the boot ROM spin? It wants us to set up the reset context! (And it wanted
        // to give us a chance to re-establish debugging after it took it away from us.)
        //
        // We assume that the user wants reset into the program they store within FlexSPI. We
        // emulate the behaviors of the boot ROM here: find the reset handler, and prepare the
        // CM7 to run that reset handler. It's convenient that the boot ROM prepares the FlexSPI
        // controller...
        //
        // But that's not always true: if you change your boot fuses, your board's boot pins, etc.
        // then the boot ROM respects that configuration. It might not initialize the FlexSPI
        // controller, and we won't be able to find the reset handler. We're not sure what to do
        // here, so we'll keep the CM7 in the boot ROM.
        //
        // (A generous tool might inspect the boot fuses to figure out what the next step would
        // be. Maybe it could invoke more boot ROM APIs to put us into the next stage. Sorry,
        // we're not yet a generous tool.)
        if let Some(pc) = self.find_flexspi_image_reset_handler(probe)? {
            self.write_core_reg(probe, PC, pc)?
        } else {
            tracing::warn!(
                "Could not find a valid reset handler in FlexSPI! Keeping the CM7 in the boot ROM."
            );
        }

        debug_cache.restore(probe)?;

        // We're halted in order to establish the reset context. Did the user want us to stay halted?
        if !self.simulate_reset_catch.load(Ordering::Relaxed) {
            self.halt(probe, false)?;
        }

        Ok(())
    }
}

/// Debug sequences for MIMXRT118x MCUs.
///
/// Currently only supports the Cortex M33. In fact, if you try to interact with the Cortex M7,
/// you'll have a bad time: its access port doesn't appear until it's released from reset!
/// For the time being, you can only do things through the CM33.
#[derive(Debug)]
pub struct MIMXRT118x(());

impl MIMXRT118x {
    const DMA4_TDC0_CH_CSR: u64 = 0x52010000;

    fn new() -> Self {
        Self(())
    }

    /// Create a sequence handle for the MIMXRT117x.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self::new())
    }

    fn halt_m33(
        &self,
        interface: &mut ArmCommunicationInterface<Initialized>,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        const SRC_SBMR2: u64 = 0x54460044;

        let mut m33 =
            ADIMemoryInterface::new(interface, &FullyQualifiedApAddress::v1_with_dp(dp, 3))?;
        let boot_mode = (m33.read_word_32(SRC_SBMR2)? >> 24) & 0x3f;
        if boot_mode == 8 || boot_mode == 9 {
            tracing::info!("Boot from fuse or serial download mode, check if CM33 is halted...");
            let cm33_dhcsr_f = Dhcsr(m33.read_word_32(Dhcsr::get_mmio_address())?);
            if !cm33_dhcsr_f.c_halt() {
                tracing::info!(
                    "CM33 is not halted, trying to halt it. CM33 DHCSR = 0x{:x}",
                    cm33_dhcsr_f.0
                );
                // Enable CM33 debug control
                m33.write_word_32(Dhcsr::get_mmio_address(), 0xA05F0001)?;
                // Halt CM33
                m33.write_word_32(Dhcsr::get_mmio_address(), 0xA05F0003)?;
                let cm33_dhcsr_s = Dhcsr(m33.read_word_32(Dhcsr::get_mmio_address())?);
                if cm33_dhcsr_s.c_halt() {
                    tracing::info!("CM33 is halted now. CM33 DHCSR = 0x{:x}", cm33_dhcsr_s.0);
                } else {
                    tracing::warn!(
                        "CM33 is still running, halt failed. CM33 DHCSR = 0x{:x}",
                        cm33_dhcsr_s.0
                    );
                }
            } else {
                tracing::info!(
                    "CM33 is halted already. CM33 DHCSR = 0x{:x}",
                    cm33_dhcsr_f.0
                );
            }
        } else {
            tracing::info!("Flash execution mode, leave CM33 run status as it was...");
        }
        Ok(())
    }

    /// Start the M7 core to make its AP available.
    fn start_cm7(
        &self,
        interface: &mut ArmCommunicationInterface<Initialized>,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        const BLK_CTRL_S_AONMIX_M7_CFG: u64 = 0x544F0080;
        const ROSC400M_CTRL1: u64 = 0x54484350;
        const CLOCK_ROOT0_CONTROL: u64 = 0x54450000;
        const SRC_GENERAL_REG_SCR: u64 = 0x54460010;
        const MU_RT_S3MUA_TR0: u64 = 0x57540200;
        const MU_RT_S3MUA_RR0: u64 = 0x57540280;
        const MU_RT_S3MUA_RR1: u64 = 0x57540284;

        // The AP for the M7 is not available, so use the one for the M33.
        let mut m33 =
            ADIMemoryInterface::new(interface, &FullyQualifiedApAddress::v1_with_dp(dp, 3))?;
        let reg = m33.read_word_32(BLK_CTRL_S_AONMIX_M7_CFG)?;
        if reg & 0x10 == 0 {
            tracing::info!("CM7 is running already");
        } else {
            tracing::info!("Start to kickoff CM7");

            // Clock Preparation
            m33.write_word_32(ROSC400M_CTRL1, 0x0)?;
            m33.write_word_32(CLOCK_ROOT0_CONTROL, 0x100)?;

            // VTOR 0x00
            m33.write_word_32(BLK_CTRL_S_AONMIX_M7_CFG, 0x0010)?;

            // Release CM7
            m33.write_word_32(SRC_GENERAL_REG_SCR, 0x1)?;

            // DMA initialization
            self.init_cm7_tcm(&mut m33)?;

            // Making Landing Zone
            m33.write_word_32(0x303C0000, 0x20000000)?;
            m33.write_word_32(0x303C0004, 0x00000009)?;
            m33.write_word_32(0x303C0008, 0xE7FEE7FE)?;

            // Trigger ELE
            m33.write_word_32(MU_RT_S3MUA_TR0, 0x17d20106)?;
            let resp1 = m33.read_word_32(MU_RT_S3MUA_RR0)?;
            let resp2 = m33.read_word_32(MU_RT_S3MUA_RR1)?;
            tracing::debug!("RESP1 = 0x{resp1:x}");
            tracing::debug!("RESP2 = 0x{resp2:x}");

            // Deassert CM7 Wait
            m33.write_word_32(BLK_CTRL_S_AONMIX_M7_CFG, 0x0)?;

            tracing::info!("CM7 is kicked-off");
        }
        Ok(())
    }

    fn init_cm7_tcm(&self, m33: &mut impl ArmMemoryInterface) -> Result<(), ArmError> {
        let reg = m33.read_word_32(Self::DMA4_TDC0_CH_CSR)?;
        if reg & 0x80000000 != 0 {
            self.wait_dma4_finished(m33)?;
        }
        self.start_dma4(m33, 0x303C0000)?;
        self.wait_dma4_finished(m33)?;
        self.start_dma4(m33, 0x30400000)?;
        self.wait_dma4_finished(m33)?;
        Ok(())
    }

    fn wait_dma4_finished(&self, m33: &mut impl ArmMemoryInterface) -> Result<(), ArmError> {
        let start = Instant::now();
        loop {
            let reg = m33.read_word_32(Self::DMA4_TDC0_CH_CSR)?;
            if reg & 0x40000000 == 0 {
                break;
            }
            if start.elapsed() >= Duration::from_secs(1) {
                return Err(ArmError::Timeout);
            }
        }
        self.clear_dma4_done(m33)
    }

    fn start_dma4(
        &self,
        m33: &mut impl ArmMemoryInterface,
        target_addr: u32,
    ) -> Result<(), ArmError> {
        const DMA4_TCD0_SLAST_SGA: u64 = 0x5201002C;
        const DMA4_TCD0_DLAST_SGA: u64 = 0x52010038;
        const DMA4_TCD0_SADDR: u64 = 0x52010020;
        const DMA4_TCD0_DADDR: u64 = 0x52010030;
        const DMA4_TCD0_NBYTES_MLOFFNO: u64 = 0x52010028;
        const DMA4_TCD0_ELINKNO: u64 = 0x52010036;
        const DMA4_TCD0_BITER_ELINKNO: u64 = 0x5201003E;
        const DMA4_TCD0_ATTR: u64 = 0x52010026;
        const DMA4_TCD0_SOFF: u64 = 0x52010024;
        const DMA4_TCD0_DOFF: u64 = 0x52010034;
        const DMA4_TCD0_CSR: u64 = 0x5201003C;

        self.clear_dma4_done(m33)?;
        m33.write_word_32(DMA4_TCD0_SLAST_SGA, 0x00000000)?;
        m33.write_word_32(DMA4_TCD0_DLAST_SGA, 0x00000000)?;
        self.clear_dma4_done(m33)?;
        m33.write_word_32(DMA4_TCD0_SADDR, 0x20484000)?;
        m33.write_word_32(DMA4_TCD0_DADDR, target_addr)?;
        m33.write_word_32(DMA4_TCD0_NBYTES_MLOFFNO, 0x40000)?;
        m33.write_word_16(DMA4_TCD0_ELINKNO, 0x1)?;
        m33.write_word_16(DMA4_TCD0_BITER_ELINKNO, 0x1)?;
        m33.write_word_16(DMA4_TCD0_ATTR, 0x0303)?;
        m33.write_word_16(DMA4_TCD0_SOFF, 0x0)?;
        m33.write_word_16(DMA4_TCD0_DOFF, 0x8)?;
        m33.write_word_32(Self::DMA4_TDC0_CH_CSR, 0x7)?;
        m33.write_word_16(DMA4_TCD0_CSR, 0x8)?;
        m33.write_word_16(DMA4_TCD0_CSR, 0x9)?;

        Ok(())
    }

    fn clear_dma4_done(&self, m33: &mut impl ArmMemoryInterface) -> Result<(), ArmError> {
        m33.write_word_32(Self::DMA4_TDC0_CH_CSR, 0x40000000)
    }
}

impl ArmDebugSequence for MIMXRT118x {
    fn debug_port_start(
        &self,
        interface: &mut ArmCommunicationInterface<Initialized>,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        // Switch to DP Register Bank 0
        interface.write_dp_register(dp, Select(0))?;

        let ctrl = interface.read_dp_register::<Ctrl>(dp)?;
        let powered_down = !(ctrl.csyspwrupack() && ctrl.cdbgpwrupack());
        if powered_down {
            tracing::info!("Debug port {dp:x?} is powered down, powering up");
            let mut ctrl = Ctrl(0);
            ctrl.set_cdbgpwrupreq(true);
            ctrl.set_csyspwrupreq(true);
            interface.write_dp_register(dp, ctrl)?;

            let start = Instant::now();
            loop {
                let ctrl = interface.read_dp_register::<Ctrl>(dp)?;
                if ctrl.csyspwrupack() && ctrl.cdbgpwrupack() {
                    break;
                }
                if start.elapsed() >= Duration::from_secs(1) {
                    return Err(ArmError::Timeout);
                }
            }
        }

        // Init AP Transfer Mode, Transaction Counter, and Lane Mask (Normal Transfer Mode, Include all Byte Lanes)
        let mut ctrl = Ctrl(0);
        ctrl.set_cdbgpwrupreq(true);
        ctrl.set_csyspwrupreq(true);
        ctrl.set_mask_lane(0b1111);
        interface.write_dp_register(dp, ctrl)?;

        // Clear errors, but don't DAPABORT.
        let mut abort = Abort(0);
        abort.set_orunerrclr(true);
        abort.set_wderrclr(true);
        abort.set_stkerrclr(true);
        abort.set_stkcmpclr(true);
        interface.write_dp_register(dp, abort)?;

        self.halt_m33(interface, dp)?;
        self.start_cm7(interface, dp)?;

        Ok(())
    }

    fn allowed_access_ports(&self) -> Vec<u8> {
        // AP5 locks the whole DP if you try to read its IDR.
        vec![0, 1, 2, 3, 4, 6]
    }
}

/// Cache the debug state of the MCU.
///
/// Some targets will lose this state once they execute a system reset. For
/// targets that know this will happen, we can restore the context after
/// the reset occurs.
///
/// There's probably more we could cache, but this is a good enough starting
/// point for 1170 testing.
///
/// The FPB assumes the v2 architecture revision, and it only cares about
/// control and comparator registers. (No caching of any CoreSight IDs.)
/// A portable implementation may need to specialize this for the FPB revision
/// of the chip.
struct DebugCache {
    fp_ctrl: FpCtrl,
    fp_comps: Vec<FpRev2CompX>,
}

impl DebugCache {
    /// Produce a debug cache from the target.
    fn from_target(probe: &mut dyn ArmMemoryInterface) -> Result<Self, ArmError> {
        let fp_ctrl = FpCtrl(probe.read_word_32(FpCtrl::get_mmio_address())?);

        Ok(Self {
            fp_ctrl,
            fp_comps: (0..fp_ctrl.num_code())
                .map(|base_address| -> Result<FpRev2CompX, ArmError> {
                    let address = FpRev2CompX::get_mmio_address_from_base(base_address as u64 * 4)?;
                    let fp_comp = probe.read_word_32(address)?;
                    Ok(FpRev2CompX(fp_comp))
                })
                .collect::<Result<Vec<_>, _>>()?,
        })
    }

    /// Put this cached debug state back into the target.
    fn restore(mut self, probe: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
        self.fp_ctrl.set_key(true);
        probe.write_word_32(FpCtrl::get_mmio_address(), self.fp_ctrl.into())?;

        for (base, fp_comp) in self.fp_comps.into_iter().enumerate() {
            probe.write_word_32(
                FpRev2CompX::get_mmio_address_from_base(base as u64 * 4)?,
                fp_comp.into(),
            )?;
        }

        Ok(())
    }
}
