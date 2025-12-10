use axplat::irq::{HandlerTable, IpiTarget, IrqHandler, IrqIf};
use loongArch64::register::{
    ecfg::{self, LineBasedInterrupt},
    ticlr,
};

use crate::config::devices::{EIOINTC_IRQ, IPI_IRQ, TIMER_IRQ};

// TODO: move these modules to a separate crate
mod eiointc;
mod pch_pic;

/// The maximum number of IRQs.
pub const MAX_IRQ_COUNT: usize = 13;

const IOCSR_IPI_SEND_CPU_SHIFT: u32 = 16;
const IOCSR_IPI_SEND_BLOCKING: u32 = 1 << 31;

const IOCSR_IPI_STATUS: u32 = 0x1000;
const IOCSR_IPI_ENABLE: u32 = 0x1004;
const IOCSR_IPI_CLEAR: u32 = 0x100c;
const IOCSR_IPI_SEND: u32 = 0x1040;

#[inline(always)]
fn read_iocsr(reg: u32) -> u32 {
    let val: u32;
    unsafe {
        core::arch::asm!(
            "iocsrrd.w {}, {}",
            out(reg) val,
            in(reg) reg,
            options(nostack, nomem)
        );
    }
    val
}

#[inline(always)]
fn write_iocsr(reg: u32, val: u32) {
    unsafe {
        core::arch::asm!(
            "iocsrwr.w {}, {}",
            in(reg) val,
            in(reg) reg,
            options(nostack)
        );
    }
}

static IRQ_HANDLER_TABLE: HandlerTable<MAX_IRQ_COUNT> = HandlerTable::new();

pub(crate) fn init() {
    eiointc::init();
    pch_pic::init();
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IrqType {
    Timer,
    Io,
    Ipi,
    Ex(usize),
}

impl IrqType {
    fn new(irq: usize) -> Self {
        match irq {
            TIMER_IRQ => Self::Timer,
            EIOINTC_IRQ => Self::Io,
            IPI_IRQ => Self::Ipi,
            n => Self::Ex(n),
        }
    }

    fn as_usize(&self) -> usize {
        match self {
            IrqType::Timer => TIMER_IRQ,
            IrqType::Io => EIOINTC_IRQ,
            IrqType::Ipi => IPI_IRQ,
            IrqType::Ex(n) => *n,
        }
    }
}

struct IrqIfImpl;

#[impl_plat_interface]
impl IrqIf for IrqIfImpl {
    /// Enables or disables the given IRQ.
    fn set_enable(irq: usize, enabled: bool) {
        let irq = IrqType::new(irq);
        match irq {
            IrqType::Timer | IrqType::Ipi => {
                let core_local_irq = match irq {
                    IrqType::Timer => Some(LineBasedInterrupt::TIMER),
                    IrqType::Ipi => {
                        write_iocsr(IOCSR_IPI_ENABLE, u32::MAX);
                        Some(LineBasedInterrupt::IPI)
                    }
                    _ => {
                        warn!("unsupported IRQ type for core-local interrupt");
                        None
                    }
                };
                if let Some(interrupt_bit) = core_local_irq {
                    let old_value = ecfg::read().lie();
                    let new_value = match enabled {
                        true => old_value | interrupt_bit,
                        false => old_value & !interrupt_bit,
                    };
                    ecfg::set_lie(new_value);
                }
            }
            IrqType::Io => {}
            IrqType::Ex(irq) => {
                if enabled {
                    eiointc::enable_irq(irq);
                    pch_pic::enable_irq(irq);
                } else {
                    eiointc::disable_irq(irq);
                    pch_pic::disable_irq(irq);
                }
            }
        }
    }

    /// Registers an IRQ handler for the given IRQ.
    fn register(irq: usize, handler: IrqHandler) -> bool {
        if IRQ_HANDLER_TABLE.register_handler(irq, handler) {
            Self::set_enable(irq, true);
            return true;
        }
        warn!("register handler for IRQ {} failed", irq);
        false
    }

    /// Unregisters the IRQ handler for the given IRQ.
    ///
    /// It also disables the IRQ if the unregistration succeeds. It returns the
    /// existing handler if it is registered, `None` otherwise.
    fn unregister(irq: usize) -> Option<IrqHandler> {
        IRQ_HANDLER_TABLE
            .unregister_handler(irq)
            .inspect(|_| Self::set_enable(irq, false))
    }

    /// Handles the IRQ.
    ///
    /// It is called by the common interrupt handler. It should look up in the
    /// IRQ handler table and calls the corresponding handler. If necessary, it
    /// also acknowledges the interrupt controller after handling.
    fn handle(irq: usize) -> Option<usize> {
        let mut irq = IrqType::new(irq);

        if matches!(irq, IrqType::Io) {
            let Some(ex_irq) = eiointc::claim_irq() else {
                debug!("Spurious external IRQ");
                return None;
            };
            irq = IrqType::Ex(ex_irq);
        }

        trace!("IRQ {irq:?}");

        if !IRQ_HANDLER_TABLE.handle(irq.as_usize()) {
            debug!("Unhandled IRQ {irq:?}");
        }

        match irq {
            IrqType::Timer => {
                ticlr::clear_timer_interrupt();
            }
            IrqType::Ipi => write_iocsr(IOCSR_IPI_CLEAR, 0x1),
            IrqType::Io => {}
            IrqType::Ex(irq) => {
                eiointc::complete_irq(irq);
            }
        }

        Some(irq.as_usize())
    }

    /// Sends an inter-processor interrupt (IPI) to the specified target CPU or all CPUs.
    fn send_ipi(_irq_num: usize, target: IpiTarget) {
        match target {
            IpiTarget::Current { cpu_id } => {
                write_iocsr(
                    IOCSR_IPI_SEND,
                    (cpu_id as u32) << IOCSR_IPI_SEND_CPU_SHIFT | IOCSR_IPI_SEND_BLOCKING,
                );
            }
            IpiTarget::Other { cpu_id } => {
                write_iocsr(
                    IOCSR_IPI_SEND,
                    (cpu_id as u32) << IOCSR_IPI_SEND_CPU_SHIFT | IOCSR_IPI_SEND_BLOCKING,
                );
            }
            IpiTarget::AllExceptCurrent { cpu_id, cpu_num } => {
                for i in 0..cpu_num {
                    if i != cpu_id {
                        write_iocsr(
                            IOCSR_IPI_SEND,
                            (i as u32) << IOCSR_IPI_SEND_CPU_SHIFT | IOCSR_IPI_SEND_BLOCKING,
                        );
                    }
                }
            }
        }
    }
}
