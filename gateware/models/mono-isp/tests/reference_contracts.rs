use mono_isp::{
    DEFECT_DELTA_MAX, DefectEntry, DefectMask, HISTOGRAM_COUNT_MAX, MonoIspConfig,
    MonoIspReference, RAW_MAX, TONE_LUT_ENTRIES, build_tone_lut, pack_defect_table, registers,
    unpack_defect_table,
};

#[test]
fn black_level_and_gamma_are_one_raw_indexed_lookup() {
    let gamma = std::array::from_fn::<_, TONE_LUT_ENTRIES, _>(|index| {
        (((index as u32 * index as u32) + 1023) / 4092).min(255) as u8
    });
    let lut = build_tone_lut(64, &gamma).unwrap();

    for raw in 0..=RAW_MAX {
        let corrected = raw.saturating_sub(64);
        assert_eq!(lut[usize::from(raw)], gamma[usize::from(corrected)]);
    }
    assert!(build_tone_lut(RAW_MAX + 1, &gamma).is_err());
}

#[test]
fn cumulative_defects_cover_boundaries_and_continuations() {
    let entries = vec![
        DefectEntry::new(0, DefectMask::Lane0),
        DefectEntry::new(DEFECT_DELTA_MAX, DefectMask::Lane1),
        DefectEntry::new(2 * DEFECT_DELTA_MAX + 7, DefectMask::Both),
    ];
    let packed = pack_defect_table(&entries, 1024, 1025).unwrap();
    assert_eq!(
        packed.words,
        [
            0x1,
            (DEFECT_DELTA_MAX << 2) | 0x2,
            DEFECT_DELTA_MAX << 2,
            (7 << 2) | 0x3,
        ]
    );
    assert_eq!(
        unpack_defect_table(&packed.words, 1024, 1025).unwrap(),
        entries
    );

    assert!(pack_defect_table(&[entries[1], entries[0]], 1024, 1025).is_err());
    assert!(pack_defect_table(&[entries[0], entries[0]], 1024, 1025).is_err());
    assert!(unpack_defect_table(&[1 << 20], 1024, 1025).is_err());
}

#[test]
fn correction_precedes_histogram_and_tone_mapping() {
    let tone = std::array::from_fn(|raw| (255 - (raw >> 2)) as u8);
    let config = MonoIspConfig::new(8, 2)
        .with_tone_lut(tone)
        .with_lut_enabled(true)
        .with_histogram_enabled(true)
        .with_defects(vec![
            DefectEntry::new(0, DefectMask::Lane0),
            DefectEntry::new(1, DefectMask::Lane1),
            DefectEntry::new(2, DefectMask::Both),
            DefectEntry::new(4, DefectMask::Both),
        ]);
    let mut reference = MonoIspReference::new(config).unwrap();
    reference.clear_histogram();
    let input = [
        10, 100, 200, 300, 400, 500, 600, 700, // row zero
        800, 900, 20, 30, 40, 50, 60, 70, // row one
    ];
    let output = reference.process_frame(&input).unwrap();

    let corrected: [u16; 16] = [
        100, 100, 200, 200, 200, 200, 600, 700, // all three masks
        0, 0, 20, 30, 40, 50, 60, 70, // row-start BOTH resets history
    ];
    let expected = corrected.map(|raw| tone[usize::from(raw)]);
    assert_eq!(output, expected);

    for pair in corrected.chunks_exact(2) {
        let average = (u32::from(pair[0]) + u32::from(pair[1]) + 1) >> 1;
        let bin = (average >> 1) as usize;
        assert!(reference.histogram()[bin] >= 1);
    }
}

#[test]
fn histogram_repeated_bin_accumulation_saturates() {
    let config = MonoIspConfig::new(2, 1).with_histogram_enabled(true);
    let mut reference = MonoIspReference::new(config).unwrap();
    assert!(reference.process_frame(&[400, 401]).is_err());
    reference.clear_histogram();

    for _ in 0..=HISTOGRAM_COUNT_MAX {
        reference.process_frame(&[400, 401]).unwrap();
    }
    assert_eq!(reference.histogram()[200], HISTOGRAM_COUNT_MAX);
    reference.process_frame(&[400, 401]).unwrap();
    assert_eq!(reference.histogram()[200], HISTOGRAM_COUNT_MAX);

    reference.clear_histogram();
    assert!(reference.histogram().iter().all(|&count| count == 0));
}

#[test]
fn wishbone_windows_cover_every_table_word_without_overlap() {
    assert_eq!(registers::tone_lut_address(0), Some(0x1000));
    assert_eq!(registers::tone_lut_address(1023), Some(0x1ffc));
    assert_eq!(registers::tone_lut_address(1024), None);
    assert_eq!(registers::defect_table_address(0), Some(0x2000));
    assert_eq!(registers::defect_table_address(511), Some(0x27fc));
    assert_eq!(registers::defect_table_address(512), None);
    let tone_last = registers::TONE_LUT_LAST;
    let defect_base = registers::DEFECT_TABLE_BASE;
    assert!(tone_last < defect_base);
}
