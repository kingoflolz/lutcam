//! Child-key ordering and bounded-list policy proofs ported from pytest.

use lrcca::{
    ChildArenaReferenceModel, full_log_mass_child_sort_key, log_m_q5_3, tagdetect_child_sort_key,
};

const MAX_LIST_LEN: usize = 9;

#[derive(Clone, Copy, Debug)]
struct LegalKey {
    area: u8,
    dense: u16,
    coarse: u8,
}

fn dense_key(area_key: u8, log_m_q5_3: u16) -> u16 {
    (u16::from(area_key) - 1) * 128 + log_m_q5_3
}

fn coarse_key(dense: u16) -> u8 {
    (dense >> 7) as u8
}

fn area_from_coarse_key(coarse: u8) -> u8 {
    if coarse >= 6 { 7 } else { coarse + 1 }
}

fn sign(value: i32) -> i8 {
    match value.cmp(&0) {
        std::cmp::Ordering::Less => -1,
        std::cmp::Ordering::Equal => 0,
        std::cmp::Ordering::Greater => 1,
    }
}

fn legal_dense_keys() -> Vec<LegalKey> {
    let mut keys = Vec::with_capacity(1024);
    for area in 1..=7 {
        let log_limit = if area < 7 { 128 } else { 256 };
        for log_m in 0..log_limit {
            let dense = dense_key(area, log_m);
            keys.push(LegalKey {
                area,
                dense,
                coarse: coarse_key(dense),
            });
        }
    }
    keys
}

#[test]
fn coarse_key_is_dense_rank_top_bits_and_preserves_strict_area_order() {
    let legal = legal_dense_keys();
    assert_eq!(legal.len(), 1024);
    for left in &legal {
        assert_eq!(left.coarse, coarse_key(left.dense));
        assert_eq!(area_from_coarse_key(left.coarse), left.area);
        for right in &legal {
            if left.area != right.area {
                assert_eq!(
                    sign(i32::from(left.coarse) - i32::from(right.coarse)),
                    sign(i32::from(left.area) - i32::from(right.area)),
                );
            }
            if left.coarse != right.coarse {
                assert_eq!(
                    sign(i32::from(left.dense) - i32::from(right.dense)),
                    sign(i32::from(left.coarse) - i32::from(right.coarse)),
                );
            }
        }
    }
}

#[test]
fn reference_key_uses_saturated_area_seven_log_split() {
    for area in 0_usize..32 {
        let area_key = area.min(7);
        for mass in [0, 1, 65_535, 65_536, 1 << 30] {
            let expected = match area_key {
                0 => 0,
                1..=6 => (area_key - 1) as u8,
                _ => 6 + u8::from(mass >= 65_536),
            };
            assert_eq!(tagdetect_child_sort_key(area, mass), expected);
        }
    }
}

#[test]
fn full_mass_key_is_the_centroid_compressors_complete_q5_3_value() {
    for mass in [0, 1, 2, 3, 255, 256, 65_535, 65_536, 1 << 30, u64::MAX] {
        assert_eq!(full_log_mass_child_sort_key(mass), log_m_q5_3(mass));
    }
}

#[test]
fn equal_coarse_key_add_policy_never_scans_farther_than_dense_rank() {
    let legal = legal_dense_keys();
    for existing in &legal {
        for candidate in &legal {
            let old_scan_stops = existing.dense > candidate.dense;
            let coarse_scan_stops = existing.coarse >= candidate.coarse;
            if old_scan_stops {
                assert!(coarse_scan_stops);
            }
            if old_scan_stops != coarse_scan_stops {
                assert_eq!(existing.coarse, candidate.coarse);
            }

            let old_head_or_reuse_stops = candidate.dense <= existing.dense;
            let coarse_head_or_reuse_stops = candidate.coarse <= existing.coarse;
            if old_head_or_reuse_stops {
                assert!(coarse_head_or_reuse_stops);
            }
            if old_head_or_reuse_stops != coarse_head_or_reuse_stops {
                assert_eq!(existing.coarse, candidate.coarse);
            }
        }
    }
}

fn merge_service_cost_for_sequence(c_len: usize, l_len: usize, sequence: &[bool]) -> usize {
    let mut remaining = [c_len, l_len];
    let mut indices = [0, 0];
    let target = (c_len + l_len).min(MAX_LIST_LEN);
    let mut drop_remaining = (c_len + l_len).saturating_sub(MAX_LIST_LEN);
    let mut output_count = 0;
    let mut pending_node = None;
    let mut spare_node = None;
    let mut canonical_dropped = false;
    let mut carry_valid = false;
    let mut cycles = 1; // S_MERGE_DIRECT_SETUP

    for &canonical_source in sequence {
        cycles += 1; // S_MERGE_DIRECT_SELECT
        let source = usize::from(!canonical_source);
        let node = (source, indices[source]);
        let has_next = remaining[source] != 1;
        let is_canonical_head = node == (0, 0);

        if drop_remaining != 0 {
            if is_canonical_head {
                canonical_dropped = true;
            }
            drop_remaining -= 1;
            indices[source] += 1;
            remaining[source] -= 1;
            if has_next {
                cycles += 1; // S_MERGE_DIRECT_ADVANCE
            }
            continue;
        }

        let first_output = output_count == 0;
        let use_carry = !first_output && carry_valid && is_canonical_head;
        let use_spare = !first_output && !use_carry && spare_node.is_some();
        let output_node = if first_output {
            (0, 0)
        } else if use_carry || use_spare {
            spare_node.expect("carry/spare selection requires a saved node")
        } else {
            node
        };
        let pending_needs_write = pending_node.is_some() && pending_node != Some((0, 0));
        pending_node = Some(output_node);
        output_count += 1;

        if first_output {
            if node != (0, 0) {
                spare_node = Some(node);
                carry_valid = !canonical_dropped;
            } else {
                spare_node = None;
                carry_valid = false;
            }
        } else if use_carry {
            spare_node = None;
            carry_valid = false;
        } else if use_spare {
            spare_node = (node != (0, 0)).then_some(node);
        }

        indices[source] += 1;
        remaining[source] -= 1;
        if has_next {
            if pending_needs_write {
                cycles += 1; // S_MERGE_DIRECT_ADVANCE_REQ
            }
            cycles += 1; // S_MERGE_DIRECT_ADVANCE
        }
    }

    assert_eq!(output_count, target);
    assert_eq!(drop_remaining, 0);
    assert_eq!(remaining, [0, 0]);
    cycles += 1; // S_MERGE_DIRECT_WRITE_TAIL
    if spare_node.is_some() {
        cycles += 1; // S_MERGE_DIRECT_FREE_SPARE
    }
    cycles + 1 // S_MERGE_DIRECT_WRITE_HEAD
}

fn coarse_equal_service_min_sequence(c_len: usize, l_len: usize) -> Vec<bool> {
    let mut remaining = [c_len, l_len];
    let mut drop_remaining = (c_len + l_len).saturating_sub(MAX_LIST_LEN);
    let mut sequence = Vec::with_capacity(c_len + l_len);
    while remaining != [0, 0] {
        let source = if remaining[0] == 0 {
            1
        } else if remaining[1] == 0 || (drop_remaining == 0 && remaining[0] >= remaining[1]) {
            0
        } else {
            1
        };
        sequence.push(source == 0);
        remaining[source] -= 1;
        drop_remaining = drop_remaining.saturating_sub(1);
    }
    sequence
}

fn visit_combinations(
    total: usize,
    choose: usize,
    start: usize,
    chosen: &mut Vec<usize>,
    callback: &mut impl FnMut(&[usize]),
) {
    if chosen.len() == choose {
        callback(chosen);
        return;
    }
    let needed = choose - chosen.len();
    for position in start..=total - needed {
        chosen.push(position);
        visit_combinations(total, choose, position + 1, chosen, callback);
        chosen.pop();
    }
}

#[test]
fn equal_coarse_merge_policy_is_minimum_service_for_every_old_tie_order() {
    for c_len in 1..=MAX_LIST_LEN {
        for l_len in 1..=MAX_LIST_LEN {
            let total = c_len + l_len;
            let mut minimum = usize::MAX;
            visit_combinations(
                total,
                c_len,
                0,
                &mut Vec::with_capacity(c_len),
                &mut |canonical_positions| {
                    let mut sequence = vec![false; total];
                    for &position in canonical_positions {
                        sequence[position] = true;
                    }
                    minimum = minimum.min(merge_service_cost_for_sequence(c_len, l_len, &sequence));
                },
            );
            let new_sequence = coarse_equal_service_min_sequence(c_len, l_len);
            assert_eq!(
                merge_service_cost_for_sequence(c_len, l_len, &new_sequence),
                minimum,
                "c_len={c_len} l_len={l_len}",
            );
        }
    }
}

fn merge_service_cost_for_keys(
    canonical: &[(u8, u16)],
    old: &[(u8, u16)],
    coarse_only: bool,
) -> usize {
    let mut c_index = 0;
    let mut l_index = 0;
    let mut drop_remaining = (canonical.len() + old.len()).saturating_sub(MAX_LIST_LEN);
    let mut sequence = Vec::with_capacity(canonical.len() + old.len());
    while c_index < canonical.len() || l_index < old.len() {
        let canonical_source = if c_index >= canonical.len() {
            false
        } else if l_index >= old.len() {
            true
        } else {
            let c_key = if coarse_only {
                u16::from(canonical[c_index].0)
            } else {
                canonical[c_index].1
            };
            let l_key = if coarse_only {
                u16::from(old[l_index].0)
            } else {
                old[l_index].1
            };
            if c_key < l_key {
                true
            } else if l_key < c_key {
                false
            } else if !coarse_only {
                true
            } else if drop_remaining != 0 {
                false
            } else {
                canonical.len() - c_index >= old.len() - l_index
            }
        };
        sequence.push(canonical_source);
        if canonical_source {
            c_index += 1;
        } else {
            l_index += 1;
        }
        drop_remaining = drop_remaining.saturating_sub(1);
    }
    merge_service_cost_for_sequence(canonical.len(), old.len(), &sequence)
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

    fn range(&mut self, start: u64, end: u64) -> u64 {
        start + self.next() % (end - start)
    }
}

fn random_key_list(rng: &mut Rng) -> Vec<(u8, u16)> {
    let length = rng.range(1, 10) as usize;
    let mut result = Vec::with_capacity(length);
    for _ in 0..length {
        let area = rng.range(1, 8) as u8;
        let log_limit = if area < 7 { 128 } else { 256 };
        let dense = dense_key(area, rng.range(0, log_limit) as u16);
        result.push((coarse_key(dense), dense));
    }
    result.sort_unstable();
    result
}

#[test]
fn mixed_coarse_merge_service_is_never_worse_for_seeded_cases() {
    let mut rng = Rng(0xa578);
    for case in 0..20_000 {
        let canonical = random_key_list(&mut rng);
        let old = random_key_list(&mut rng);
        assert!(
            merge_service_cost_for_keys(&canonical, &old, true)
                <= merge_service_cost_for_keys(&canonical, &old, false),
            "case={case} canonical={canonical:?} old={old:?}",
        );
    }
}

fn combinations_with_replacement(
    keys: &[(u8, u16)],
    start: usize,
    remaining: usize,
    current: &mut Vec<(u8, u16)>,
    output: &mut Vec<Vec<(u8, u16)>>,
) {
    if remaining == 0 {
        output.push(current.clone());
        return;
    }
    for index in start..keys.len() {
        current.push(keys[index]);
        combinations_with_replacement(keys, index, remaining - 1, current, output);
        current.pop();
    }
}

#[test]
fn two_bin_two_rank_merge_service_is_exhaustively_never_worse() {
    let fine_keys = [(0, 0), (0, 1), (1, 2), (1, 3)];
    let mut lists = Vec::new();
    for length in 1..=MAX_LIST_LEN {
        combinations_with_replacement(
            &fine_keys,
            0,
            length,
            &mut Vec::with_capacity(length),
            &mut lists,
        );
    }
    assert_eq!(lists.len(), 714);
    for canonical in &lists {
        for old in &lists {
            assert!(
                merge_service_cost_for_keys(canonical, old, true)
                    <= merge_service_cost_for_keys(canonical, old, false),
                "canonical={canonical:?} old={old:?}",
            );
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct KeyedFeature {
    key: u8,
    identity: u16,
}

fn coarse_arena() -> ChildArenaReferenceModel<KeyedFeature> {
    ChildArenaReferenceModel::<KeyedFeature>::with_sort_key(64, MAX_LIST_LEN, true, |feature| {
        feature.key
    })
    .unwrap()
}

#[test]
fn more_than_nine_children_emit_the_largest_nine_and_free_the_list() {
    let keys = [11, 3, 17, 13, 9, 16, 12, 15, 14, 10, 17, 11, 2, 16, 20, 1];
    let mut arena = coarse_arena();
    let head = arena.allocate().unwrap();
    for (identity, key) in keys.into_iter().enumerate() {
        let feature = KeyedFeature {
            key,
            identity: identity as u16,
        };
        if identity == 0 {
            arena.init(head, feature).unwrap();
        } else {
            arena.add(head, feature).unwrap();
        }
        arena.check_invariants().unwrap();
    }
    let retained = arena.read(head).unwrap().features;
    assert_eq!(
        retained
            .iter()
            .map(|feature| feature.key)
            .collect::<Vec<_>>(),
        vec![12, 13, 14, 15, 16, 16, 17, 17, 20],
    );
    assert_eq!(arena.free_count, arena.entry_count);
    assert!(arena.valid_heads().is_empty());
    arena.check_invariants().unwrap();
}

#[test]
fn equal_children_beyond_nine_do_not_replace_retained_ties() {
    let mut arena = coarse_arena();
    let head = arena.allocate().unwrap();
    arena
        .init(
            head,
            KeyedFeature {
                key: 42,
                identity: 0,
            },
        )
        .unwrap();
    for identity in 1..MAX_LIST_LEN as u16 {
        arena.add(head, KeyedFeature { key: 42, identity }).unwrap();
    }
    let initial = arena.features(head).unwrap();
    assert_eq!(
        initial
            .iter()
            .map(|feature| feature.identity)
            .collect::<Vec<_>>(),
        vec![0, 8, 7, 6, 5, 4, 3, 2, 1],
    );
    for identity in MAX_LIST_LEN as u16..100 {
        arena.add(head, KeyedFeature { key: 42, identity }).unwrap();
        assert_eq!(arena.features(head).unwrap(), initial);
    }
    assert_eq!(arena.read(head).unwrap().features, initial);
    assert_eq!(arena.free_count, arena.entry_count);
    arena.check_invariants().unwrap();
}

#[test]
fn coarse_policy_keeps_largest_n_and_preserves_unequal_key_children() {
    let mut rng = Rng(0xa578a);
    for case in 0..250 {
        let features = (0..30)
            .map(|identity| KeyedFeature {
                key: rng.range(1, 8) as u8,
                identity,
            })
            .collect::<Vec<_>>();
        let mut arena = coarse_arena();
        let head = arena.allocate().unwrap();
        arena.init(head, features[0]).unwrap();
        for &feature in &features[1..] {
            arena.add(head, feature).unwrap();
        }

        let retained = arena.features(head).unwrap();
        let retained_keys = retained
            .iter()
            .map(|feature| feature.key)
            .collect::<Vec<_>>();
        let mut expected_keys = features
            .iter()
            .map(|feature| feature.key)
            .collect::<Vec<_>>();
        expected_keys.sort_unstable();
        let expected_keys = &expected_keys[expected_keys.len() - MAX_LIST_LEN..];
        assert_eq!(retained_keys, expected_keys, "case={case}");

        let boundary = expected_keys[0];
        for required in features.iter().filter(|feature| feature.key > boundary) {
            assert!(
                retained.contains(required),
                "case={case} missing {required:?}"
            );
        }
        assert!(retained_keys.windows(2).all(|pair| pair[0] <= pair[1]));
        arena.check_invariants().unwrap();
    }
}

#[test]
fn coarse_policy_merge_keeps_largest_n_and_sorted_keys() {
    let mut rng = Rng(0xa578b);
    for case in 0..250 {
        let left_features = (0..18)
            .map(|identity| KeyedFeature {
                key: rng.range(1, 8) as u8,
                identity,
            })
            .collect::<Vec<_>>();
        let right_features = (0..18)
            .map(|identity| KeyedFeature {
                key: rng.range(1, 8) as u8,
                identity: 100 + identity,
            })
            .collect::<Vec<_>>();
        let mut arena = coarse_arena();
        let left = arena.allocate().unwrap();
        let right = arena.allocate().unwrap();
        arena.init(left, left_features[0]).unwrap();
        arena.init(right, right_features[0]).unwrap();
        for &feature in &left_features[1..] {
            arena.add(left, feature).unwrap();
        }
        for &feature in &right_features[1..] {
            arena.add(right, feature).unwrap();
        }
        let canonical = arena.merge(Some(left), Some(right)).unwrap().unwrap();

        let retained = arena.features(canonical).unwrap();
        let retained_keys = retained
            .iter()
            .map(|feature| feature.key)
            .collect::<Vec<_>>();
        let all_features = left_features
            .iter()
            .chain(&right_features)
            .copied()
            .collect::<Vec<_>>();
        let mut expected_keys = all_features
            .iter()
            .map(|feature| feature.key)
            .collect::<Vec<_>>();
        expected_keys.sort_unstable();
        let expected_keys = &expected_keys[expected_keys.len() - MAX_LIST_LEN..];
        assert_eq!(retained_keys, expected_keys, "case={case}");

        let boundary = expected_keys[0];
        for required in all_features.iter().filter(|feature| feature.key > boundary) {
            assert!(
                retained.contains(required),
                "case={case} missing {required:?}"
            );
        }
        assert!(retained_keys.windows(2).all(|pair| pair[0] <= pair[1]));
        arena.check_invariants().unwrap();
    }
}
