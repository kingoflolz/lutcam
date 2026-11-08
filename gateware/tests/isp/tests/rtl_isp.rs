//! End-to-end Marlin/Verilator checks for the ten-bit monochrome ISP.

use std::path::{Path, PathBuf};

use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use mono_isp::{
    DefectEntry, DefectMask, HISTOGRAM_BINS, MonoIspConfig, MonoIspReference, build_tone_lut,
    pack_defect_table, registers,
};
use rtl_test_support::{TestResult, clock, read_u64};

const WIDTH: usize = 16;
const HEIGHT: usize = 4;
const PAIRS_PER_ROW: usize = WIDTH / 2;
const PAIR_COUNT: usize = PAIRS_PER_ROW * HEIGHT;
const ADAPTER: &str = "tests/isp/tests/rtl_adapters/mono_isp_small.sv";
const MANIFEST: &str = "rtl/isp/sources.f";

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("wb_cyc_i", 0, 0, PortDirection::Input),
    ("wb_stb_i", 0, 0, PortDirection::Input),
    ("wb_we_i", 0, 0, PortDirection::Input),
    ("wb_sel_i", 3, 0, PortDirection::Input),
    ("wb_adr_i", 15, 0, PortDirection::Input),
    ("wb_dat_i", 31, 0, PortDirection::Input),
    ("wb_dat_o", 31, 0, PortDirection::Output),
    ("wb_ack_o", 0, 0, PortDirection::Output),
    ("wb_err_o", 0, 0, PortDirection::Output),
    ("in_valid", 0, 0, PortDirection::Input),
    ("in_ready", 0, 0, PortDirection::Output),
    ("in_pixel0", 9, 0, PortDirection::Input),
    ("in_pixel1", 9, 0, PortDirection::Input),
    ("in_sof", 0, 0, PortDirection::Input),
    ("in_eol", 0, 0, PortDirection::Input),
    ("out_valid", 0, 0, PortDirection::Output),
    ("out_ready", 0, 0, PortDirection::Input),
    ("out_pixel0", 7, 0, PortDirection::Output),
    ("out_pixel1", 7, 0, PortDirection::Output),
    ("out_sof", 0, 0, PortDirection::Output),
    ("out_eol", 0, 0, PortDirection::Output),
    ("hist_dump_req", 0, 0, PortDirection::Input),
    ("hist_dump_ready", 0, 0, PortDirection::Output),
    ("hist_out_valid", 0, 0, PortDirection::Output),
    ("hist_out_ready", 0, 0, PortDirection::Input),
    ("hist_out_data", 31, 0, PortDirection::Output),
    ("protocol_error", 0, 0, PortDirection::Output),
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct OutputPair {
    d0: u8,
    d1: u8,
    sof: bool,
    eol: bool,
}

#[derive(Clone, Copy, Debug)]
struct ConcurrentWriteResult {
    ack: bool,
    err: bool,
    input_ready: bool,
    dump_ready: bool,
}

#[derive(Clone, Copy)]
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut value = self.0;
        value ^= value >> 12;
        value ^= value << 25;
        value ^= value >> 27;
        self.0 = value;
        value.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
}

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn with_isp(
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult,
) -> TestResult {
    let root = gateware_root();
    let adapter = root.join(ADAPTER);
    rtl_test_support::with_model(
        &root,
        Path::new(MANIFEST),
        "mono-isp-small",
        "MonoIspSmall",
        Path::new(ADAPTER),
        PORTS,
        &[Path::new("rtl/isp")],
        &[adapter.as_path()],
        test,
    )
}

fn pin_idle_bus(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("wb_cyc_i", 0_u8)?;
    model.pin("wb_stb_i", 0_u8)?;
    model.pin("wb_we_i", 0_u8)?;
    model.pin("wb_sel_i", 0_u8)?;
    model.pin("wb_adr_i", 0_u16)?;
    model.pin("wb_dat_i", 0_u32)?;
    Ok(())
}

fn pin_idle_streams(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("in_valid", 0_u8)?;
    model.pin("in_pixel0", 0_u16)?;
    model.pin("in_pixel1", 0_u16)?;
    model.pin("in_sof", 0_u8)?;
    model.pin("in_eol", 0_u8)?;
    model.pin("out_ready", 1_u8)?;
    model.pin("hist_dump_req", 0_u8)?;
    model.pin("hist_out_ready", 0_u8)?;
    Ok(())
}

fn reset(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    pin_idle_bus(model)?;
    pin_idle_streams(model)?;
    model.pin("rst", 1_u8)?;
    for _ in 0..4 {
        clock(model)?;
    }
    model.pin("rst", 0_u8)?;
    clock(model)
}

fn concurrent_write_edge(
    model: &mut DynamicVerilatedModel<'_>,
    address: u16,
    data: u32,
) -> TestResult<ConcurrentWriteResult> {
    model.pin("wb_cyc_i", 1_u8)?;
    model.pin("wb_stb_i", 1_u8)?;
    model.pin("wb_we_i", 1_u8)?;
    model.pin("wb_sel_i", 0xf_u8)?;
    model.pin("wb_adr_i", address)?;
    model.pin("wb_dat_i", data)?;
    model.pin("clk", 0_u8)?;
    model.eval();
    let result = ConcurrentWriteResult {
        ack: false,
        err: false,
        input_ready: read_u64(model, "in_ready")? != 0,
        dump_ready: read_u64(model, "hist_dump_ready")? != 0,
    };

    model.pin("clk", 1_u8)?;
    model.eval();
    let result = ConcurrentWriteResult {
        ack: read_u64(model, "wb_ack_o")? != 0,
        err: read_u64(model, "wb_err_o")? != 0,
        ..result
    };
    model.pin("clk", 0_u8)?;
    model.eval();
    pin_idle_bus(model)?;
    model.eval();
    assert_ne!(result.ack, result.err, "concurrent write did not terminate");
    Ok(result)
}

fn abort_delayed_ram_read(
    model: &mut DynamicVerilatedModel<'_>,
    aborted_address: u16,
    next_address: u16,
    next_value: u32,
) -> TestResult {
    model.pin("wb_cyc_i", 1_u8)?;
    model.pin("wb_stb_i", 1_u8)?;
    model.pin("wb_we_i", 0_u8)?;
    model.pin("wb_sel_i", 0xf_u8)?;
    model.pin("wb_adr_i", aborted_address)?;
    model.pin("wb_dat_i", 0_u32)?;
    model.pin("clk", 0_u8)?;
    model.eval();
    model.pin("clk", 1_u8)?;
    model.eval();
    assert_eq!(read_u64(model, "wb_ack_o")?, 0, "RAM read ACKed early");
    assert_eq!(read_u64(model, "wb_err_o")?, 0, "RAM read errored early");
    model.pin("clk", 0_u8)?;
    model.eval();

    // Withdraw the classic bus request before its registered RAM response.
    pin_idle_bus(model)?;
    model.eval();
    for _ in 0..3 {
        clock(model)?;
        assert_eq!(
            read_u64(model, "wb_ack_o")?,
            0,
            "aborted RAM read produced a phantom ACK"
        );
        assert_eq!(
            read_u64(model, "wb_err_o")?,
            0,
            "aborted RAM read produced a phantom ERR"
        );
    }

    // A distinct request must return its own value, not the canceled port
    // response retained in an internal pending register.
    assert_eq!(wb_read(model, next_address)?, next_value);
    Ok(())
}

fn verify_run_clear_arbitration(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    wb_write(model, registers::REG_CONTROL, registers::CONTROL_RUN)?;
    wait_status(model, registers::STATUS_DEFECT_READY, 0)?;
    model.pin("in_valid", 1_u8)?;
    model.pin("in_pixel0", 400_u16)?;
    model.pin("in_pixel1", 800_u16)?;
    model.pin("in_sof", 1_u8)?;
    model.pin("in_eol", 0_u8)?;
    model.pin("out_ready", 1_u8)?;
    let write = concurrent_write_edge(model, registers::REG_CONTROL, 0)?;
    assert!(write.ack);
    model.pin("in_valid", 0_u8)?;
    model.eval();

    let mut observed = None;
    for _ in 0..8 {
        model.pin("clk", 0_u8)?;
        model.eval();
        if read_u64(model, "out_valid")? != 0 {
            observed = Some(current_output(model)?);
            clock(model)?;
            break;
        }
        clock(model)?;
    }
    if write.input_ready {
        assert_eq!(
            observed,
            Some(OutputPair {
                d0: 100,
                d1: 200,
                sof: true,
                eol: false,
            }),
            "pair accepted beside RUN clear was dropped or corrupted"
        );
    } else {
        assert_eq!(observed, None, "unaccepted pair reached the output");
    }
    wait_status(
        model,
        0,
        registers::STATUS_RUN | registers::STATUS_PIPELINE_BUSY,
    )?;
    Ok(())
}

fn assert_stalled_output(
    model: &DynamicVerilatedModel<'_>,
    expected: OutputPair,
    context: &str,
) -> TestResult {
    assert_eq!(
        read_u64(model, "out_valid")?,
        1,
        "out_valid dropped {context}"
    );
    assert_eq!(current_output(model)?, expected, "output changed {context}");
    Ok(())
}

fn verify_stalled_lut_output_survives_stop_and_read(
    model: &mut DynamicVerilatedModel<'_>,
) -> TestResult {
    let source0 = 100usize;
    let source1 = 200usize;
    let unrelated = 777usize;
    wb_write(model, registers::tone_lut_address(source0).unwrap(), 0x35)?;
    wb_write(model, registers::tone_lut_address(source1).unwrap(), 0xc9)?;
    wb_write(model, registers::tone_lut_address(unrelated).unwrap(), 0xee)?;
    wb_write(
        model,
        registers::REG_CONTROL,
        registers::CONTROL_RUN | registers::CONTROL_LUT_ENABLE,
    )?;
    wait_status(model, registers::STATUS_DEFECT_READY, 0)?;

    model.pin("out_ready", 0_u8)?;
    model.pin("in_valid", 1_u8)?;
    model.pin("in_pixel0", source0 as u16)?;
    model.pin("in_pixel1", source1 as u16)?;
    model.pin("in_sof", 1_u8)?;
    model.pin("in_eol", 0_u8)?;
    model.pin("clk", 0_u8)?;
    model.eval();
    assert_eq!(read_u64(model, "in_ready")?, 1);
    clock(model)?;
    model.pin("in_valid", 0_u8)?;
    model.eval();

    let expected = OutputPair {
        d0: 0x35,
        d1: 0xc9,
        sof: true,
        eol: false,
    };
    assert_stalled_output(model, expected, "after LUT pair acceptance")?;

    let stop = concurrent_write_edge(model, registers::REG_CONTROL, 0)?;
    assert!(stop.ack);
    assert_stalled_output(model, expected, "across RUN clear")?;

    // A stopped but still-busy pipeline must not lend the tone RAM port to
    // Wishbone: reading the unrelated 0xee entry would otherwise overwrite
    // tone_port0_q and corrupt the stalled 0x35 payload.
    model.pin("wb_cyc_i", 1_u8)?;
    model.pin("wb_stb_i", 1_u8)?;
    model.pin("wb_we_i", 0_u8)?;
    model.pin("wb_sel_i", 0xf_u8)?;
    model.pin("wb_adr_i", registers::tone_lut_address(unrelated).unwrap())?;
    model.pin("wb_dat_i", 0_u32)?;
    model.pin("clk", 0_u8)?;
    model.eval();
    assert_stalled_output(model, expected, "before busy tone read")?;
    model.pin("clk", 1_u8)?;
    model.eval();
    assert_eq!(read_u64(model, "wb_ack_o")?, 0);
    assert_eq!(read_u64(model, "wb_err_o")?, 1);
    model.pin("clk", 0_u8)?;
    model.eval();
    pin_idle_bus(model)?;
    model.eval();
    assert_stalled_output(model, expected, "across rejected busy tone read")?;

    model.pin("out_ready", 1_u8)?;
    clock(model)?;
    assert_eq!(read_u64(model, "out_valid")?, 0);
    wait_status(
        model,
        0,
        registers::STATUS_RUN | registers::STATUS_PIPELINE_BUSY,
    )?;
    Ok(())
}

fn initialize_histogram(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    wb_write(
        model,
        registers::REG_COMMAND,
        registers::COMMAND_HISTOGRAM_CLEAR,
    )?;
    wait_status(
        model,
        registers::STATUS_HISTOGRAM_INITIALIZED,
        registers::STATUS_HISTOGRAM_CLEAR_BUSY,
    )?;
    Ok(())
}

fn verify_dump_control_arbitration(
    model: &mut DynamicVerilatedModel<'_>,
    address: u16,
    data: u32,
    conflicting_status: u32,
) -> TestResult {
    initialize_histogram(model)?;
    model.pin("hist_dump_req", 1_u8)?;
    model.pin("hist_out_ready", 0_u8)?;
    let write = concurrent_write_edge(model, address, data)?;
    let dump_accepted = write.dump_ready;
    model.pin("hist_dump_req", 0_u8)?;
    model.eval();

    // Each advertised handshake must be honored.  It is never legal to ACK
    // the control write while also advertising acceptance of the dump.
    assert!(!(dump_accepted && write.ack));
    let status = wb_read(model, registers::REG_STATUS)?;
    let dump_busy = status & registers::STATUS_HISTOGRAM_DUMP_BUSY != 0;
    let control_busy = status & conflicting_status != 0;
    assert!(
        !(dump_busy && control_busy),
        "dump and control operation entered conflicting states"
    );
    if dump_accepted {
        assert!(dump_busy);
    }
    if write.ack {
        assert!(control_busy);
    }
    Ok(())
}

fn wb_access(
    model: &mut DynamicVerilatedModel<'_>,
    address: u16,
    write: Option<u32>,
) -> TestResult<(bool, u32)> {
    model.pin("wb_cyc_i", 1_u8)?;
    model.pin("wb_stb_i", 1_u8)?;
    model.pin("wb_we_i", u8::from(write.is_some()))?;
    model.pin("wb_sel_i", 0xf_u8)?;
    model.pin("wb_adr_i", address)?;
    model.pin("wb_dat_i", write.unwrap_or(0))?;

    for _ in 0..32 {
        model.pin("clk", 0_u8)?;
        model.eval();
        model.pin("clk", 1_u8)?;
        model.eval();
        let ack = read_u64(model, "wb_ack_o")? != 0;
        let err = read_u64(model, "wb_err_o")? != 0;
        let data = read_u64(model, "wb_dat_o")? as u32;
        model.pin("clk", 0_u8)?;
        model.eval();
        if ack || err {
            assert_ne!(ack, err, "Wishbone ACK and ERR were asserted together");
            pin_idle_bus(model)?;
            model.eval();
            return Ok((err, data));
        }
    }
    Err(format!("Wishbone transaction at {address:#06x} did not terminate").into())
}

fn wb_write(model: &mut DynamicVerilatedModel<'_>, address: u16, data: u32) -> TestResult {
    let (error, _) = wb_access(model, address, Some(data))?;
    if error {
        return Err(format!("Wishbone write at {address:#06x} returned ERR").into());
    }
    Ok(())
}

fn wb_read(model: &mut DynamicVerilatedModel<'_>, address: u16) -> TestResult<u32> {
    let (error, data) = wb_access(model, address, None)?;
    if error {
        return Err(format!("Wishbone read at {address:#06x} returned ERR").into());
    }
    Ok(data)
}

fn wb_write_error(model: &mut DynamicVerilatedModel<'_>, address: u16, data: u32) -> TestResult {
    let (error, _) = wb_access(model, address, Some(data))?;
    assert!(error, "Wishbone write at {address:#06x} unexpectedly ACKed");
    Ok(())
}

fn wb_read_error(model: &mut DynamicVerilatedModel<'_>, address: u16) -> TestResult {
    let (error, _) = wb_access(model, address, None)?;
    assert!(error, "Wishbone read at {address:#06x} unexpectedly ACKed");
    Ok(())
}

fn wait_status(model: &mut DynamicVerilatedModel<'_>, set: u32, clear: u32) -> TestResult<u32> {
    for _ in 0..2048 {
        let status = wb_read(model, registers::REG_STATUS)?;
        if status & set == set && status & clear == 0 {
            return Ok(status);
        }
    }
    Err(format!("status did not reach set={set:#x}, clear={clear:#x}").into())
}

fn current_output(model: &DynamicVerilatedModel<'_>) -> TestResult<OutputPair> {
    Ok(OutputPair {
        d0: read_u64(model, "out_pixel0")? as u8,
        d1: read_u64(model, "out_pixel1")? as u8,
        sof: read_u64(model, "out_sof")? != 0,
        eol: read_u64(model, "out_eol")? != 0,
    })
}

fn stream_frame(
    model: &mut DynamicVerilatedModel<'_>,
    input: &[[u16; 2]],
    expected_pixels: &[u8],
    seed: u64,
    require_full_rate_prefix: bool,
) -> TestResult {
    assert_eq!(input.len(), PAIR_COUNT);
    assert_eq!(expected_pixels.len(), 2 * PAIR_COUNT);
    let expected = expected_pixels
        .chunks_exact(2)
        .enumerate()
        .map(|(index, pair)| OutputPair {
            d0: pair[0],
            d1: pair[1],
            sof: index == 0,
            eol: (index + 1) % PAIRS_PER_ROW == 0,
        })
        .collect::<Vec<_>>();

    pin_idle_bus(model)?;
    model.pin("hist_dump_req", 0_u8)?;
    model.pin("hist_out_ready", 0_u8)?;
    let mut rng = Rng(seed);
    let mut input_index = 0usize;
    let mut output = Vec::with_capacity(PAIR_COUNT);
    let mut presenting = false;
    let mut stalled = None;

    for cycle in 0..20_000 {
        if !presenting && input_index < input.len() {
            // Keep the first three transfers contiguous.  Directed histogram
            // vectors use these cycles to exercise read/modify/write
            // forwarding on one repeated bin; later transfers retain random
            // input bubbles.
            presenting = input_index < 3 || rng.next() & 3 != 0;
        }
        if presenting {
            let pair = input[input_index];
            model.pin("in_valid", 1_u8)?;
            model.pin("in_pixel0", pair[0])?;
            model.pin("in_pixel1", pair[1])?;
            model.pin("in_sof", u8::from(input_index == 0))?;
            model.pin(
                "in_eol",
                u8::from((input_index + 1).is_multiple_of(PAIRS_PER_ROW)),
            )?;
        } else {
            model.pin("in_valid", 0_u8)?;
            model.pin("in_pixel0", 0_u16)?;
            model.pin("in_pixel1", 0_u16)?;
            model.pin("in_sof", 0_u8)?;
            model.pin("in_eol", 0_u8)?;
        }
        let output_ready = input_index < 3 || rng.next() & 3 != 0;
        model.pin("out_ready", u8::from(output_ready))?;
        model.pin("clk", 0_u8)?;
        model.eval();

        let output_valid = read_u64(model, "out_valid")? != 0;
        if let Some(previous) = stalled {
            assert!(
                output_valid,
                "out_valid dropped under backpressure at cycle {cycle}"
            );
            assert_eq!(current_output(model)?, previous, "stalled output changed");
        }
        let visible = output_valid.then(|| current_output(model)).transpose()?;
        if output_valid && output_ready {
            output.push(visible.expect("valid output has a payload"));
        }
        stalled = if output_valid && !output_ready {
            visible
        } else {
            None
        };

        let input_ready = read_u64(model, "in_ready")? != 0;
        if require_full_rate_prefix && input_index < 3 {
            assert!(
                presenting && input_ready && output_ready,
                "consecutive defect prefix bubbled at pair {input_index} on cycle {cycle}"
            );
        }
        let input_accepted = presenting && input_ready;
        model.pin("clk", 1_u8)?;
        model.eval();
        model.pin("clk", 0_u8)?;
        model.eval();
        if input_accepted {
            input_index += 1;
            presenting = false;
        }

        if input_index == input.len() && output.len() == expected.len() && stalled.is_none() {
            break;
        }
        if output.len() > expected.len() {
            return Err("ISP emitted more pairs than the frame contains".into());
        }
    }

    assert_eq!(
        input_index,
        input.len(),
        "ISP did not accept the whole frame"
    );
    assert_eq!(output, expected, "RTL/reference frame mismatch");

    pin_idle_streams(model)?;
    for _ in 0..8 {
        model.pin("clk", 0_u8)?;
        model.eval();
        assert_eq!(read_u64(model, "out_valid")?, 0, "unexpected frame tail");
        clock(model)?;
    }
    Ok(())
}

fn dump_histogram(model: &mut DynamicVerilatedModel<'_>, expected: &[u32]) -> TestResult {
    assert_eq!(expected.len(), HISTOGRAM_BINS);
    pin_idle_bus(model)?;
    model.pin("in_valid", 0_u8)?;
    model.pin("out_ready", 1_u8)?;
    model.pin("hist_dump_req", 1_u8)?;
    let mut requested = false;
    let mut records = Vec::with_capacity(HISTOGRAM_BINS);
    let mut stalled = None;
    let mut rng = Rng(0xada9_02b7_e151_39d1);

    for cycle in 0..20_000 {
        let ready = rng.next() & 3 != 0;
        model.pin("hist_out_ready", u8::from(ready))?;
        model.pin("clk", 0_u8)?;
        model.eval();

        if !requested && read_u64(model, "hist_dump_ready")? != 0 {
            requested = true;
        }
        let valid = read_u64(model, "hist_out_valid")? != 0;
        let data = read_u64(model, "hist_out_data")? as u32;
        if let Some(previous) = stalled {
            assert!(
                valid,
                "hist_out_valid dropped while stalled at cycle {cycle}"
            );
            assert_eq!(data, previous, "histogram record changed while stalled");
        }
        if valid && ready {
            records.push(data);
        }
        stalled = (valid && !ready).then_some(data);

        model.pin("clk", 1_u8)?;
        model.eval();
        model.pin("clk", 0_u8)?;
        model.eval();
        if requested {
            model.pin("hist_dump_req", 0_u8)?;
        }
        if records.len() == HISTOGRAM_BINS && stalled.is_none() {
            break;
        }
    }
    model.pin("hist_dump_req", 0_u8)?;
    model.pin("hist_out_ready", 0_u8)?;
    assert!(requested, "histogram dump request was never accepted");
    assert_eq!(records, expected, "histogram dump differs from reference");
    Ok(())
}

#[test]
fn wishbone_stream_defects_tone_and_histogram_match_reference() -> TestResult {
    with_isp(|model| {
        reset(model)?;
        assert_eq!(wb_read(model, registers::REG_ID)?, registers::ISP_ID);
        wb_write_error(model, registers::REG_ID, 0)?;
        wb_read_error(model, 0x0002)?;
        wb_read_error(model, 0x0040)?;

        wb_write(model, registers::tone_lut_address(0).unwrap(), 0x3c)?;
        wb_write(model, registers::tone_lut_address(1).unwrap(), 0xa7)?;
        abort_delayed_ram_read(
            model,
            registers::tone_lut_address(0).unwrap(),
            registers::tone_lut_address(1).unwrap(),
            0xa7,
        )?;
        wb_write(model, registers::defect_table_address(0).unwrap(), 0x12345)?;
        wb_write(model, registers::defect_table_address(1).unwrap(), 0x2abcd)?;
        abort_delayed_ram_read(
            model,
            registers::defect_table_address(0).unwrap(),
            registers::defect_table_address(1).unwrap(),
            0x2abcd,
        )?;

        verify_stalled_lut_output_survives_stop_and_read(model)?;
        reset(model)?;
        verify_run_clear_arbitration(model)?;
        reset(model)?;
        verify_dump_control_arbitration(
            model,
            registers::REG_CONTROL,
            registers::CONTROL_RUN,
            registers::STATUS_RUN,
        )?;
        reset(model)?;
        verify_dump_control_arbitration(
            model,
            registers::REG_COMMAND,
            registers::COMMAND_HISTOGRAM_CLEAR,
            registers::STATUS_HISTOGRAM_CLEAR_BUSY,
        )?;
        reset(model)?;

        // Reset defaults to the raw-truncation bypass and an empty defect table.
        let bypass_pairs = (0..PAIR_COUNT)
            .map(|index| {
                [
                    ((index * 67 + 3) & 0x3ff) as u16,
                    ((index * 149 + 511) & 0x3ff) as u16,
                ]
            })
            .collect::<Vec<_>>();
        let bypass_pixels = bypass_pairs
            .iter()
            .flat_map(|pair| pair.map(|pixel| (pixel >> 2) as u8))
            .collect::<Vec<_>>();
        wb_write(model, registers::REG_CONTROL, registers::CONTROL_RUN)?;
        wait_status(model, registers::STATUS_DEFECT_READY, 0)?;
        stream_frame(
            model,
            &bypass_pairs,
            &bypass_pixels,
            0x81af_37de_02ab_9951,
            false,
        )?;
        wb_write(model, registers::REG_CONTROL, 0)?;

        // Program the cumulative representation directly so mask-zero skip
        // words are verified in RTL without requiring a 262k-pair frame.  A
        // delta-three continuation followed by delta one resolves a lane-zero
        // defect at pair address four.
        let continuation_words = [(3_u32 << 2), (1_u32 << 2) | 1];
        for (index, &word) in continuation_words.iter().enumerate() {
            wb_write(model, registers::defect_table_address(index).unwrap(), word)?;
        }
        wb_write(
            model,
            registers::REG_DEFECT_COUNT,
            continuation_words.len() as u32,
        )?;
        let continuation_pairs = (0..PAIR_COUNT)
            .map(|index| {
                [
                    ((index * 31) & 0x3ff) as u16,
                    ((index * 31 + 17) & 0x3ff) as u16,
                ]
            })
            .collect::<Vec<_>>();
        let continuation_config = MonoIspConfig::new(WIDTH, HEIGHT);
        let mut continuation_reference =
            MonoIspReference::from_packed_defects(continuation_config, &continuation_words)?;
        let continuation_expected = continuation_reference
            .process_frame_pairs(&continuation_pairs)?
            .into_iter()
            .flatten()
            .collect::<Vec<_>>();
        wb_write(model, registers::REG_CONTROL, registers::CONTROL_RUN)?;
        wait_status(model, registers::STATUS_DEFECT_READY, 0)?;
        stream_frame(
            model,
            &continuation_pairs,
            &continuation_expected,
            0x60fc_bfa2_579d_02c1,
            false,
        )?;
        wb_write(model, registers::REG_CONTROL, 0)?;

        let gamma = std::array::from_fn(|index| {
            (((index as u32 * 29) ^ ((index as u32) >> 2) ^ 0xa5) & 0xff) as u8
        });
        let tone_lut = build_tone_lut(73, &gamma)?;
        for (index, &value) in tone_lut.iter().enumerate() {
            wb_write(
                model,
                registers::tone_lut_address(index).unwrap(),
                u32::from(value),
            )?;
        }
        for index in [0, 1, 72, 73, 511, 1023] {
            assert_eq!(
                wb_read(model, registers::tone_lut_address(index).unwrap())?,
                u32::from(tone_lut[index])
            );
        }

        let defects = vec![
            DefectEntry::new(0, DefectMask::Lane0),
            DefectEntry::new(1, DefectMask::Lane1),
            DefectEntry::new(2, DefectMask::Both),
            DefectEntry::new(8, DefectMask::Both),
            DefectEntry::new(9, DefectMask::Lane0),
            DefectEntry::new(31, DefectMask::Lane1),
        ];
        let packed = pack_defect_table(&defects, WIDTH, HEIGHT)?;
        for (index, &word) in packed.words.iter().enumerate() {
            wb_write(model, registers::defect_table_address(index).unwrap(), word)?;
            assert_eq!(
                wb_read(model, registers::defect_table_address(index).unwrap())?,
                word
            );
        }
        wb_write(model, registers::REG_DEFECT_COUNT, packed.len() as u32)?;
        wb_write(
            model,
            registers::REG_COMMAND,
            registers::COMMAND_HISTOGRAM_CLEAR,
        )?;
        wait_status(
            model,
            registers::STATUS_HISTOGRAM_INITIALIZED,
            registers::STATUS_HISTOGRAM_CLEAR_BUSY,
        )?;

        let input = (0..PAIR_COUNT)
            .map(|index| match index {
                // The first three corrected pairs are all [400, 400] under
                // Lane0, Lane1, and Both masks respectively.  The stream
                // driver accepts them on consecutive cycles, forcing the
                // histogram's same-bin forwarding path.
                0 => [123, 400],
                1 => [400, 777],
                2 => [1, 2],
                _ => [
                    ((index * 97 + 21) & 0x3ff) as u16,
                    ((index * 211 + 901) & 0x3ff) as u16,
                ],
            })
            .collect::<Vec<_>>();
        let config = MonoIspConfig::new(WIDTH, HEIGHT)
            .with_tone_lut(tone_lut)
            .with_lut_enabled(true)
            .with_histogram_enabled(true)
            .with_defects(defects);
        let mut reference = MonoIspReference::new(config)?;
        reference.clear_histogram();
        let expected = reference.process_frame_pairs(&input)?;
        assert!(reference.histogram()[200] >= 3);
        let expected_pixels = expected.into_iter().flatten().collect::<Vec<_>>();

        let control = registers::CONTROL_RUN
            | registers::CONTROL_LUT_ENABLE
            | registers::CONTROL_HISTOGRAM_ENABLE;
        wb_write(model, registers::REG_CONTROL, control)?;
        wait_status(model, registers::STATUS_DEFECT_READY, 0)?;
        wb_write_error(model, registers::tone_lut_address(0).unwrap(), 0)?;
        wb_write_error(model, registers::defect_table_address(0).unwrap(), 0)?;
        wb_write_error(model, registers::REG_DEFECT_COUNT, 0)?;
        wb_write_error(
            model,
            registers::REG_COMMAND,
            registers::COMMAND_HISTOGRAM_CLEAR,
        )?;
        stream_frame(model, &input, &expected_pixels, 0x6ca8_04f3_42de_7119, true)?;
        wb_write(model, registers::REG_CONTROL, 0)?;
        wait_status(
            model,
            0,
            registers::STATUS_RUN | registers::STATUS_PIPELINE_BUSY,
        )?;
        dump_histogram(model, &reference.histogram_dump())?;
        wait_status(model, 0, registers::STATUS_HISTOGRAM_DUMP_BUSY)?;

        // A malformed first transfer is sticky and blocks further input until
        // RUN is cleared; software can then clear the diagnostic explicitly.
        wb_write(model, registers::REG_DEFECT_COUNT, 0)?;
        wb_write(model, registers::REG_CONTROL, registers::CONTROL_RUN)?;
        wait_status(model, registers::STATUS_DEFECT_READY, 0)?;
        model.pin("in_valid", 1_u8)?;
        model.pin("in_pixel0", 12_u16)?;
        model.pin("in_pixel1", 34_u16)?;
        model.pin("in_sof", 0_u8)?;
        model.pin("in_eol", 0_u8)?;
        model.pin("out_ready", 1_u8)?;
        for _ in 0..16 {
            clock(model)?;
            if read_u64(model, "in_ready")? == 0 {
                break;
            }
        }
        model.pin("in_valid", 0_u8)?;
        wait_status(model, registers::STATUS_PROTOCOL_ERROR, 0)?;
        assert_eq!(read_u64(model, "in_ready")?, 0);
        wb_write(model, registers::REG_CONTROL, 0)?;
        wb_write(
            model,
            registers::REG_COMMAND,
            registers::COMMAND_PROTOCOL_ERROR_CLEAR,
        )?;
        wait_status(model, 0, registers::STATUS_PROTOCOL_ERROR)?;
        Ok(())
    })
}
