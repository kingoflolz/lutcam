use lrcca::{
    CallbackFeatureModel, TwoPpcInputs, TwoPpcReferenceModel, decode_bbox, iter_2ppc_pairs,
};

fn blank(width: usize, height: usize) -> Vec<Vec<u8>> {
    vec![vec![0; width]; height]
}

#[test]
fn streamed_retirement_lanes_and_parent_taps_match_the_reference_trace() {
    let rows = vec![
        vec![0; 8],
        vec![0, 1, 1, 0, 1, 0, 0, 0],
        vec![0, 1, 0, 0, 1, 1, 0, 0],
        vec![0; 8],
        vec![0; 8],
    ];
    let mut model = TwoPpcReferenceModel::new(8, 5, false).unwrap();
    let mut emissions = Vec::new();
    let mut taps = Vec::new();
    for (cycle, input) in iter_2ppc_pairs(&rows).unwrap().into_iter().enumerate() {
        let output = model.step(input, None).unwrap();
        if model.parent_head_taps != [None, None] {
            taps.push((cycle, model.parent_head_taps));
        }
        if output.fv_valid != 0 {
            emissions.push((cycle, output.fv_lane, decode_bbox(output.fv_data, 8, 5)));
        }
    }

    assert_eq!(
        taps,
        vec![
            (5, [Some((1, 0)), Some((1, 0))]),
            (6, [None, Some((1, 1))]),
            (9, [Some((0, 0)), None]),
            (10, [None, Some((0, 1))]),
            (11, [Some((0, 1)), None]),
        ]
    );
    assert_eq!(
        emissions,
        vec![
            (13, Some(0), (1, 2, 1, 2, 3)),
            (15, Some(0), (4, 5, 1, 2, 3)),
        ]
    );
}

#[test]
fn an_even_x_run_retires_on_lane_one_before_end_of_frame() {
    let mut rows = blank(8, 4);
    rows[1][2] = 1;
    let mut model = TwoPpcReferenceModel::new(8, 4, false).unwrap();
    let emissions = iter_2ppc_pairs(&rows)
        .unwrap()
        .into_iter()
        .enumerate()
        .filter_map(|(cycle, input)| {
            let output = model.step(input, None).unwrap();
            (output.fv_valid != 0)
                .then(|| (cycle, output.fv_lane, decode_bbox(output.fv_data, 8, 4)))
        })
        .collect::<Vec<_>>();
    assert_eq!(emissions, vec![(9, Some(1), (2, 2, 1, 1, 1))]);
}

#[test]
fn hole_fill_requires_the_documented_idle_flush_and_preserves_lane_timing() {
    let mut rows = blank(8, 5);
    rows[1][1] = 1;
    rows[2][2] = 1;
    let inputs = iter_2ppc_pairs(&rows).unwrap();
    let mut model = TwoPpcReferenceModel::new(8, 5, true).unwrap();
    let mut emissions = Vec::new();
    for (cycle, input) in inputs.into_iter().enumerate() {
        let eol = input.eol != 0;
        let output = model.step(input, None).unwrap();
        if output.fv_valid != 0 {
            emissions.push((cycle, output.fv_lane, decode_bbox(output.fv_data, 8, 5)));
        }
        if eol {
            let output = model.step(TwoPpcInputs::default(), None).unwrap();
            assert_eq!(output.fv_valid, 0);
        }
    }
    assert_eq!(emissions, vec![(14, Some(1), (1, 2, 1, 2, 2))]);

    let mut model = TwoPpcReferenceModel::new(8, 5, true).unwrap();
    let first_row = iter_2ppc_pairs(&rows).unwrap();
    for input in first_row.iter().take(4).copied() {
        model.step(input, None).unwrap();
    }
    assert!(
        model
            .step(first_row[4], None)
            .unwrap_err()
            .contains("flush must complete")
    );
    model.step(TwoPpcInputs::default(), None).unwrap();
    model.step(first_row[4], None).unwrap();
}

#[test]
fn custom_feature_callbacks_and_parent_head_mutation_are_streamed() {
    let callbacks = CallbackFeatureModel::new(
        0_u64,
        |_x, _y, pixel| pixel,
        |left, right| left + right,
        |feature| *feature,
    );
    let mut topology = blank(8, 4);
    topology[1][2] = 1;
    let mut features = vec![vec![0_u64; 8]; 4];
    features[1][2] = 5;
    let mut model =
        TwoPpcReferenceModel::with_feature_model(8, 4, false, callbacks.clone()).unwrap();

    let inputs = iter_2ppc_pairs(&topology).unwrap();
    let mut emitted = Vec::new();
    for (cycle, input) in inputs.into_iter().enumerate() {
        let row = cycle / 4;
        let pair = cycle % 4;
        let output = model
            .step(
                input,
                Some((features[row][2 * pair], features[row][2 * pair + 1])),
            )
            .unwrap();
        if cycle == 5 {
            assert_eq!(model.parent_head_taps, [None, Some((1, 0))]);
            assert_eq!(model.feature_for_head(1, 0).unwrap(), 5);
            model.replace_feature_for_head(1, 0, 9).unwrap();
            model.add_feature_to_head(1, 0, 100).unwrap();
        }
        if output.fv_valid != 0 {
            emitted.push((cycle, output.fv_lane, output.fv_data));
        }
    }
    assert_eq!(emitted, vec![(9, Some(1), 109)]);

    let sum_model = TwoPpcReferenceModel::with_feature_model(8, 4, false, callbacks).unwrap();
    assert_eq!(
        sum_model
            .feature_outputs(&topology, Some(&features))
            .unwrap(),
        vec![5]
    );

    let coordinate_callbacks = CallbackFeatureModel::new(
        0_u64,
        |x, y, pixel| pixel * (100 * y as u64 + x as u64),
        |left, right| left + right,
        |feature| *feature,
    );
    let mut topology = blank(8, 4);
    topology[1][1] = 1;
    topology[1][2] = 1;
    topology[2][1] = 1;
    topology[1][5] = 1;
    let mut features = vec![vec![0_u64; 8]; 4];
    features[1][1] = 2;
    features[1][2] = 3;
    features[2][1] = 5;
    features[1][5] = 7;
    let coordinate_model =
        TwoPpcReferenceModel::with_feature_model(8, 4, false, coordinate_callbacks).unwrap();
    let mut outputs = coordinate_model
        .feature_outputs(&topology, Some(&features))
        .unwrap();
    outputs.sort_unstable();
    assert_eq!(outputs, vec![735, 1_513]);
}

#[test]
fn framing_errors_are_reported_at_the_stream_boundary() {
    let mut model = TwoPpcReferenceModel::new(8, 4, false).unwrap();
    assert!(
        model
            .step(
                TwoPpcInputs {
                    datavalid: 0,
                    sof: 1,
                    ..Default::default()
                },
                None,
            )
            .unwrap_err()
            .contains("only valid with datavalid")
    );
    assert!(
        model
            .step(
                TwoPpcInputs {
                    datavalid: 1,
                    ..Default::default()
                },
                None,
            )
            .unwrap_err()
            .contains("sof must be asserted")
    );
    assert!(
        model
            .step(
                TwoPpcInputs {
                    datavalid: 1,
                    sof: 1,
                    eol: 1,
                    ..Default::default()
                },
                None,
            )
            .unwrap_err()
            .contains("eol must be asserted exactly")
    );

    model.reset();
    let rows = blank(8, 4);
    let first = iter_2ppc_pairs(&rows).unwrap()[0];
    model.step(first, None).unwrap();
    assert!(
        model
            .step(
                TwoPpcInputs {
                    datavalid: 1,
                    sof: 1,
                    ..Default::default()
                },
                None,
            )
            .unwrap_err()
            .contains("sof may only")
    );
}
