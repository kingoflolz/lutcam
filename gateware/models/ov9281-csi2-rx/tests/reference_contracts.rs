use ov9281_csi2_rx::{
    BEATS_PER_LINE, CsiHeader, DATA_TYPE_FRAME_END, DATA_TYPE_FRAME_START, DATA_TYPE_LINE_START,
    DATA_TYPE_RAW10, FRAME_BEATS, HeaderCorrection, HeaderEccError, IMAGE_HEIGHT, IMAGE_WIDTH,
    OV9281_VIRTUAL_CHANNEL, Ov9281PacketReference, ParsedPacket, RAW10_BYTES_PER_LINE, Raw10Error,
    ReceiverStatus, build_long_packet, build_short_packet, decode_header, expected_frame_beats,
    header_ecc, pack_raw10, parse_packet, payload_crc16, unpack_raw10,
};

#[derive(Clone, Copy)]
struct TestRng(u64);

impl TestRng {
    fn next_u64(&mut self) -> u64 {
        let mut value = self.0;
        value ^= value << 13;
        value ^= value >> 7;
        value ^= value << 17;
        self.0 = value;
        value
    }

    fn usize(&mut self, exclusive_maximum: usize) -> usize {
        (self.next_u64() as usize) % exclusive_maximum
    }

    fn byte(&mut self) -> u8 {
        self.next_u64() as u8
    }

    fn pixel(&mut self) -> u16 {
        (self.next_u64() & 0x03ff) as u16
    }
}

fn explicit_spec_ecc(data: [u8; 3]) -> u8 {
    let bit = |index: usize| -> u8 { (data[index / 8] >> (index % 8)) & 1 };
    let parity = [
        [0, 1, 2, 4, 5, 7, 10, 11, 13, 16, 20, 21, 22, 23].as_slice(),
        [0, 1, 3, 4, 6, 8, 10, 12, 14, 17, 20, 21, 22, 23].as_slice(),
        [0, 2, 3, 5, 6, 9, 11, 12, 15, 18, 20, 21, 22].as_slice(),
        [1, 2, 3, 7, 8, 9, 13, 14, 15, 19, 20, 21, 23].as_slice(),
        [4, 5, 6, 7, 8, 9, 16, 17, 18, 19, 20, 22, 23].as_slice(),
        [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23].as_slice(),
    ];
    parity.iter().enumerate().fold(0_u8, |ecc, (index, bits)| {
        let value = bits
            .iter()
            .fold(0_u8, |value, bit_index| value ^ bit(*bit_index));
        ecc | (value << index)
    })
}

fn flip_protected_header_bit(mut encoded: [u8; 4], bit: usize) -> [u8; 4] {
    assert!(bit < 30);
    if bit < 24 {
        encoded[bit / 8] ^= 1_u8 << (bit % 8);
    } else {
        encoded[3] ^= 1_u8 << (bit - 24);
    }
    encoded
}

#[test]
fn ecc_matches_the_six_spec_equations_for_random_headers() {
    let mut rng = TestRng(0xd4c3_b2a1_8070_6050);
    for _ in 0..20_000 {
        let data = [rng.byte(), rng.byte(), rng.byte()];
        assert_eq!(header_ecc(data), explicit_spec_ecc(data));
    }
}

#[test]
fn ecc_exhaustively_corrects_single_and_rejects_double_protected_bit_errors() {
    let headers = [
        CsiHeader {
            data_id: 0,
            data_field: 0,
        },
        CsiHeader {
            data_id: DATA_TYPE_RAW10,
            data_field: RAW10_BYTES_PER_LINE as u16,
        },
        CsiHeader {
            data_id: 0xff,
            data_field: 0xffff,
        },
        CsiHeader {
            data_id: 0x41,
            data_field: 0xa55a,
        },
    ];

    for header in headers {
        let encoded = header.encode();
        assert_eq!(
            decode_header(encoded).unwrap().correction,
            HeaderCorrection::None
        );

        for bit in 0..30 {
            let decoded = decode_header(flip_protected_header_bit(encoded, bit)).unwrap();
            assert_eq!(decoded.header, header);
            let expected = if bit < 24 {
                HeaderCorrection::DataBit(bit as u8)
            } else {
                HeaderCorrection::EccBit((bit - 24) as u8)
            };
            assert_eq!(decoded.correction, expected);
        }

        let mut double_error_count = 0;
        for first in 0..30 {
            for second in first + 1..30 {
                let corrupt =
                    flip_protected_header_bit(flip_protected_header_bit(encoded, first), second);
                assert!(matches!(
                    decode_header(corrupt),
                    Err(HeaderEccError::Uncorrectable { .. })
                ));
                double_error_count += 1;
            }
        }
        assert_eq!(double_error_count, 435);
    }

    let mut reserved = headers[1].encode();
    reserved[3] |= 0x40;
    assert_eq!(
        decode_header(reserved),
        Err(HeaderEccError::ReservedBitsSet(0x40))
    );
}

#[test]
fn corrected_header_is_used_before_packet_length_and_type_decode() {
    let payload = vec![0x5a; RAW10_BYTES_PER_LINE];
    let packet = build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &payload).unwrap();
    let original_header = parse_packet(&packet).unwrap().header();

    for bit in 0..24 {
        let mut corrupt = packet.clone();
        corrupt[..4].copy_from_slice(&flip_protected_header_bit(
            [packet[0], packet[1], packet[2], packet[3]],
            bit,
        ));
        let parsed = parse_packet(&corrupt).unwrap();
        assert_eq!(parsed.header(), original_header);
        assert_eq!(parsed.correction(), HeaderCorrection::DataBit(bit as u8));
    }
}

#[test]
fn reflected_crc_has_a_known_check_value_and_detects_random_single_bit_errors() {
    assert_eq!(payload_crc16(b"123456789"), 0x6f91);
    assert_eq!(payload_crc16(&[]), 0xffff);

    let mut rng = TestRng(0x8bad_f00d_55aa_1234);
    for iteration in 0..2_000 {
        let length = if iteration < 8 {
            [0, 1, 2, 4, 5, 1599, 1600, 65535][iteration]
        } else {
            rng.usize(2048)
        };
        let payload = (0..length).map(|_| rng.byte()).collect::<Vec<_>>();
        let packet = build_long_packet(0, DATA_TYPE_RAW10, &payload).unwrap();
        let parsed = parse_packet(&packet).unwrap();
        assert_eq!(parsed.payload(), Some(payload.as_slice()));
        assert_eq!(parsed.crc_is_valid(), Some(true));

        let mut corrupt = packet;
        let corrupt_byte = 4 + rng.usize(length + 2);
        corrupt[corrupt_byte] ^= 1_u8 << rng.usize(8);
        assert_eq!(parse_packet(&corrupt).unwrap().crc_is_valid(), Some(false));
    }
}

#[test]
fn raw10_mapping_and_random_round_trips_are_bit_exact() {
    assert_eq!(pack_raw10(&[0, 1, 2, 3]).unwrap(), [0, 0, 0, 0, 0xe4]);
    assert_eq!(
        pack_raw10(&[0x3ff, 0x000, 0x155, 0x2aa]).unwrap(),
        [0xff, 0x00, 0x55, 0xaa, 0x93]
    );

    let mut rng = TestRng(0x1234_5678_9abc_def0);
    for _ in 0..5_000 {
        let groups = 1 + rng.usize(128);
        let pixels = (0..groups * 4).map(|_| rng.pixel()).collect::<Vec<_>>();
        let bytes = pack_raw10(&pixels).unwrap();
        assert_eq!(bytes.len(), groups * 5);
        assert_eq!(unpack_raw10(&bytes).unwrap(), pixels);
    }

    assert_eq!(
        pack_raw10(&[0, 1, 2]),
        Err(Raw10Error::PixelCountNotMultipleOfFour(3))
    );
    assert_eq!(
        pack_raw10(&[0, 1, 2, 1024]),
        Err(Raw10Error::PixelOutOfRange {
            index: 3,
            value: 1024
        })
    );
    assert_eq!(
        unpack_raw10(&[0, 1, 2, 3]),
        Err(Raw10Error::ByteCountNotMultipleOfFive(4))
    );
}

fn short(data_type: u8, frame_number: u16) -> [u8; 4] {
    build_short_packet(OV9281_VIRTUAL_CHANNEL, data_type, frame_number).unwrap()
}

fn line_pixels(line: usize) -> Vec<u16> {
    (0..IMAGE_WIDTH)
        .map(|column| ((line * 37 + column * 13) & 0x03ff) as u16)
        .collect()
}

fn line_packet(line: usize) -> Vec<u8> {
    let payload = pack_raw10(&line_pixels(line)).unwrap();
    assert_eq!(payload.len(), RAW10_BYTES_PER_LINE);
    build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &payload).unwrap()
}

#[test]
fn complete_fixed_frame_produces_exact_pixels_and_markers() {
    let frame_number = 0x5a39;
    let mut reference = Ov9281PacketReference::new();
    assert!(
        reference
            .ingest_packet(&short(DATA_TYPE_FRAME_START, frame_number))
            .beats
            .is_empty()
    );

    let mut beat_count = 0;
    let mut sof_count = 0;
    let mut sol_count = 0;
    let mut eol_count = 0;
    let mut eof_count = 0;
    for line in 0..IMAGE_HEIGHT {
        let expected_pixels = line_pixels(line);
        let outcome = reference.ingest_packet(&line_packet(line));
        assert_eq!(outcome.new_status.bits(), 0);
        assert_eq!(outcome.beats.len(), BEATS_PER_LINE);
        for (beat, expected_pair) in outcome.beats.iter().zip(expected_pixels.chunks_exact(2)) {
            assert_eq!(beat.pixels, [expected_pair[0], expected_pair[1]]);
            assert_eq!(usize::from(beat.line), line);
            assert_eq!(beat.packed_data() & 0x000f_ffff, beat.packed_data());
            sof_count += usize::from(beat.sof);
            sol_count += usize::from(beat.sol);
            eol_count += usize::from(beat.eol);
            eof_count += usize::from(beat.eof);
        }
        beat_count += outcome.beats.len();
    }
    assert_eq!(beat_count, FRAME_BEATS);
    assert_eq!(
        (sof_count, sol_count, eol_count, eof_count),
        (1, 800, 800, 1)
    );

    assert!(
        reference
            .ingest_packet(&short(DATA_TYPE_FRAME_END, frame_number))
            .beats
            .is_empty()
    );
    assert_eq!(reference.status().bits(), 0);
    assert!(!reference.frame_is_active());
    assert_eq!(reference.finish_stream().bits(), 0);
}

#[test]
fn expected_frame_beats_uses_the_same_fixed_geometry_contract() {
    let pixels = (0..IMAGE_WIDTH * IMAGE_HEIGHT)
        .map(|index| (index & 0x03ff) as u16)
        .collect::<Vec<_>>();
    let beats = expected_frame_beats(&pixels).unwrap();
    assert_eq!(beats.len(), FRAME_BEATS);
    assert!(beats[0].sof && beats[0].sol);
    assert!(beats[BEATS_PER_LINE - 1].eol);
    assert!(beats[FRAME_BEATS - 1].eol && beats[FRAME_BEATS - 1].eof);
    assert_eq!(beats[0].pixels, [0, 1]);
}

#[test]
fn crc_failure_is_sticky_but_does_not_retract_the_line() {
    let mut reference = Ov9281PacketReference::new();
    reference.ingest_packet(&short(DATA_TYPE_FRAME_START, 7));
    let mut packet = line_packet(0);
    let last = packet.len() - 1;
    packet[last] ^= 0x80;
    let outcome = reference.ingest_packet(&packet);
    assert_eq!(outcome.beats.len(), BEATS_PER_LINE);
    assert!(
        outcome
            .new_status
            .contains(ReceiverStatus::PAYLOAD_CRC_ERROR)
    );
    assert!(reference.frame_is_active());
}

#[test]
fn protocol_errors_set_fixed_status_and_recover_only_at_frame_start() {
    let mut before_start = Ov9281PacketReference::new();
    assert!(before_start.ingest_packet(&line_packet(0)).beats.is_empty());
    assert!(
        before_start
            .status()
            .contains(ReceiverStatus::FRAME_SEQUENCE_ERROR)
    );

    let mut reference = Ov9281PacketReference::new();
    reference.ingest_packet(&short(DATA_TYPE_FRAME_START, 10));

    let wrong_payload = vec![0; RAW10_BYTES_PER_LINE - 1];
    let wrong_count =
        build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &wrong_payload).unwrap();
    assert!(reference.ingest_packet(&wrong_count).beats.is_empty());
    assert!(
        reference
            .status()
            .contains(ReceiverStatus::WORD_COUNT_ERROR)
    );
    assert!(!reference.frame_is_active());
    assert!(reference.ingest_packet(&line_packet(0)).beats.is_empty());

    reference.ingest_packet(&short(DATA_TYPE_FRAME_START, 11));
    assert_eq!(
        reference.ingest_packet(&line_packet(0)).beats.len(),
        BEATS_PER_LINE
    );

    let mut double_header_error = line_packet(1);
    double_header_error[0] ^= 0x03;
    assert!(
        reference
            .ingest_packet(&double_header_error)
            .beats
            .is_empty()
    );
    assert!(
        reference
            .status()
            .contains(ReceiverStatus::HEADER_ECC_UNCORRECTABLE)
    );
    assert!(reference.ingest_packet(&line_packet(2)).beats.is_empty());

    reference.ingest_packet(&short(DATA_TYPE_FRAME_START, 12));
    reference.ingest_packet(&line_packet(0));
    reference.ingest_packet(&short(DATA_TYPE_FRAME_END, 13));
    assert!(
        reference
            .status()
            .contains(ReceiverStatus::FRAME_SEQUENCE_ERROR)
    );
    assert!(
        reference
            .status()
            .contains(ReceiverStatus::LINE_COUNT_ERROR)
    );
}

#[test]
fn unexpected_and_truncated_packets_have_distinct_status() {
    let mut reference = Ov9281PacketReference::new();
    reference.ingest_packet(&short(DATA_TYPE_FRAME_START, 1));
    reference.ingest_packet(&short(DATA_TYPE_LINE_START, 0));
    assert!(
        reference
            .status()
            .contains(ReceiverStatus::UNEXPECTED_PACKET)
    );

    reference.ingest_packet(&[0, 1, 2]);
    assert!(
        reference
            .status()
            .contains(ReceiverStatus::PACKET_LENGTH_ERROR)
    );
    assert!(!reference.frame_is_active());
}

#[test]
fn corrected_headers_are_informational_and_missing_frame_end_is_detected() {
    let mut reference = Ov9281PacketReference::new();
    let mut frame_start = short(DATA_TYPE_FRAME_START, 0x1234);
    frame_start[1] ^= 1 << 5;
    let outcome = reference.ingest_packet(&frame_start);
    assert!(
        outcome
            .new_status
            .contains(ReceiverStatus::HEADER_ECC_CORRECTED)
    );
    assert!(reference.frame_is_active());

    let new_status = reference.finish_stream();
    assert!(new_status.contains(ReceiverStatus::FRAME_SEQUENCE_ERROR));
    assert!(new_status.contains(ReceiverStatus::LINE_COUNT_ERROR));
}

#[test]
fn packet_builder_and_parser_distinguish_short_and_long_packets() {
    let short_packet = short(DATA_TYPE_FRAME_START, 0xabcd);
    assert!(matches!(
        parse_packet(&short_packet).unwrap(),
        ParsedPacket::Short { .. }
    ));

    let payload = [1, 2, 3, 4, 5];
    let long_packet = build_long_packet(0, DATA_TYPE_RAW10, &payload).unwrap();
    assert!(matches!(
        parse_packet(&long_packet).unwrap(),
        ParsedPacket::Long { .. }
    ));
}
