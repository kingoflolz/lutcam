//! Directed software-reference coverage ported from the LRCCA pytest suite.

use lrcca::{HierarchicalTwoPPCReferenceModel, TwoPpcReferenceModel, sort_boxes};

#[test]
fn two_ppc_directed_diagonals_distinguish_four_and_eight_connectivity() {
    const WIDTH: usize = 48;
    const HEIGHT: usize = 36;
    let mut rows = vec![vec![0; WIDTH]; HEIGHT];
    rows[1][1] = 1;
    rows[2][2] = 1;
    rows[1][30] = 1;
    rows[2][29] = 1;

    let four = TwoPpcReferenceModel::new(WIDTH, HEIGHT, false).unwrap();
    assert_eq!(
        sort_boxes(&four.boxes(&rows).unwrap()),
        vec![
            (1, 1, 1, 1, 1),
            (2, 2, 2, 2, 1),
            (29, 29, 2, 2, 1),
            (30, 30, 1, 1, 1),
        ],
    );

    let eight = TwoPpcReferenceModel::new(WIDTH, HEIGHT, true).unwrap();
    assert_eq!(
        sort_boxes(&eight.boxes(&rows).unwrap()),
        vec![(1, 2, 1, 2, 2), (29, 30, 1, 2, 2)],
    );
}

fn blank_hierarchy_image() -> Vec<Vec<u8>> {
    vec![vec![0; 48]; 36]
}

fn draw_rect_border(rows: &mut [Vec<u8>], x0: usize, x1: usize, y0: usize, y1: usize) {
    rows[y0][x0..=x1].fill(1);
    rows[y1][x0..=x1].fill(1);
    for row in rows.iter_mut().take(y1 + 1).skip(y0) {
        row[x0] = 1;
        row[x1] = 1;
    }
}

fn directed_hierarchy_images() -> Vec<(&'static str, Vec<Vec<u8>>, usize)> {
    let mut ring = blank_hierarchy_image();
    draw_rect_border(&mut ring, 5, 12, 4, 11);

    let mut ring_with_island = blank_hierarchy_image();
    draw_rect_border(&mut ring_with_island, 6, 16, 5, 15);
    ring_with_island[10][11] = 1;

    let mut open_to_exterior = blank_hierarchy_image();
    draw_rect_border(&mut open_to_exterior, 4, 11, 4, 11);
    open_to_exterior[7][11] = 0;

    let mut two_holes = blank_hierarchy_image();
    for row in two_holes.iter_mut().take(15).skip(6) {
        row[20..34].fill(1);
    }
    for row in two_holes.iter_mut().take(10).skip(8) {
        row[23..25].fill(0);
    }
    for row in two_holes.iter_mut().take(13).skip(11) {
        row[29..31].fill(0);
    }

    let mut many_holes = blank_hierarchy_image();
    for row in many_holes.iter_mut().take(24).skip(5) {
        row[5..43].fill(1);
    }
    for (x, y) in [
        (9, 9),
        (13, 9),
        (17, 9),
        (21, 9),
        (25, 9),
        (29, 9),
        (33, 9),
        (9, 13),
        (13, 13),
        (17, 13),
        (21, 13),
    ] {
        many_holes[y][x] = 0;
    }

    vec![
        ("ring", ring, 1),
        ("ring-with-island", ring_with_island, 2),
        ("open-to-exterior", open_to_exterior, 1),
        ("two-holes", two_holes, 1),
        ("many-holes", many_holes, 1),
    ]
}

#[test]
fn hierarchy_directed_reference_cases_preserve_groups_and_child_framing() {
    let model = HierarchicalTwoPPCReferenceModel::new(48, 36).unwrap();
    for (name, rows, expected_group_count) in directed_hierarchy_images() {
        let groups = model.feature_groups(&rows, None).unwrap();
        assert_eq!(groups.len(), expected_group_count, "{name}");
        let stream = model.output_stream(&rows, None).unwrap();

        if name != "many-holes" {
            assert!(
                groups.iter().all(|group| group.inner_features.is_empty()),
                "{name}",
            );
            assert!(stream.is_empty(), "{name}");
            continue;
        }

        assert_eq!(groups[0].inner_features.len(), 9);
        assert!(
            groups[0]
                .inner_features
                .iter()
                .all(|feature| feature.area == 1),
        );
        assert_eq!(stream.len(), 9);
        assert!(stream[..8].iter().all(|event| !event.end_inner));
        assert!(stream[8].end_inner);
    }
}
