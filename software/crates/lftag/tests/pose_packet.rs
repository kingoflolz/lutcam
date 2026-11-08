use lftag::{
    FixedPose, POSE_PACKET_WORDS, PoseMathError, PosePacket, PosePacketError, derive_lambda_q10,
    pose_from_homography,
};

fn direct_pose_packet() -> [u32; POSE_PACKET_WORDS] {
    [
        0x0000_0000,
        0x8000_0000,
        0x0000_0008,
        0x0000_0000,
        0x0000_0000,
        0x0000_0000,
        0x0000_0000,
        0x0000_0000,
        0x0000_0003,
        0x0000_0001,
        0x2001_8000,
        0xfffd_a000,
        0x0000_0000,
        0x0000_0000,
        0x1fff_4000,
        0x0000_0000,
        0x0000_0000,
        0x0000_0000,
        0x2000_a000,
        0xffff_fffb,
        0xffff_fffb,
        0x0000_8007,
    ]
}

#[test]
fn production_packet_returns_the_direct_fixed_point_pose() {
    let packet = PosePacket::from_packet(&direct_pose_packet()).unwrap();
    assert_eq!(packet.mean_decode_residual_q32, 1);
    assert!(packet.weak_perspective);
    assert!(!packet.conditioning_warning);
    assert!(!packet.saturation);
    assert_eq!(
        packet.pose,
        FixedPose {
            rotation_q29: [
                0x2001_8000,
                0xfffd_a000_u32 as i32,
                0,
                0,
                0x1fff_4000,
                0,
                0,
                0,
                0x2000_a000,
            ],
            translation_q16: [-5, -5, 0x8007],
        },
    );
    assert_eq!(
        FixedPose::from_packet(&direct_pose_packet()),
        Ok(packet.pose)
    );

    let mut saturated = direct_pose_packet();
    saturated[8] |= 1 << 3;
    assert!(PosePacket::from_packet(&saturated).unwrap().saturation);
}

#[test]
fn direct_pose_packet_rejects_bad_framing() {
    let packet = direct_pose_packet();
    assert_eq!(
        PosePacket::from_packet(&packet[..POSE_PACKET_WORDS - 1]),
        Err(PosePacketError::WrongLength),
    );

    let mut packet = direct_pose_packet();
    packet[8] = 0x0000_0002;
    assert_eq!(
        PosePacket::from_packet(&packet),
        Err(PosePacketError::InvalidStatus),
    );

    let mut packet = direct_pose_packet();
    packet[8] |= 1 << 4;
    assert_eq!(
        PosePacket::from_packet(&packet),
        Err(PosePacketError::InvalidStatus),
    );
}

#[test]
fn fixed_point_pose_reference_matches_the_direct_packet_fixture() {
    let homography_common_q16 = [0x1b98, -2, -10, 0, 0x1b96, -10, 0, 0, 0x1_1400];
    assert_eq!(derive_lambda_q10(homography_common_q16), Ok(0x251e));
    assert_eq!(
        pose_from_homography(homography_common_q16, 0x0000_3333),
        Ok(PosePacket::from_packet(&direct_pose_packet()).unwrap().pose),
    );
}

#[test]
fn fixed_point_pose_reference_rejects_invalid_internal_homographies() {
    let valid = [0x1b98, -2, -10, 0, 0x1b96, -10, 0, 0, 0x1_1400];

    let mut too_wide = valid;
    too_wide[0] = 1 << 17;
    assert_eq!(
        derive_lambda_q10(too_wide),
        Err(PoseMathError::InvalidSigned18Encoding),
    );

    let mut bad_scale = valid;
    bad_scale[8] = (1 << 15) - 1;
    assert_eq!(
        derive_lambda_q10(bad_scale),
        Err(PoseMathError::InvalidHomographyScale),
    );

    let mut zero_norm = valid;
    for index in [0, 1, 3, 4, 6, 7] {
        zero_norm[index] = 0;
    }
    assert_eq!(
        derive_lambda_q10(zero_norm),
        Err(PoseMathError::InvalidLambda),
    );
}

#[test]
fn conditioning_uses_column_ratio_with_an_exact_strict_boundary() {
    // Check both column orders and the exact strict 7/8 boundary.
    for scale in [1, 1 << 24] {
        for (first, second, expected) in [(7, 8, false), (6, 8, true), (8, 8, false)] {
            for (a, b) in [(first, second), (second, first)] {
                let pose = FixedPose {
                    rotation_q29: [a * scale, 0, 0, 0, b * scale, 0, 0, 0, 0],
                    translation_q16: [0; 3],
                };
                assert_eq!(pose.conditioning_warning(), expected);
            }
        }
    }
}

#[test]
fn conditioning_comparison_handles_full_signed_packet_range() {
    let mut pose = FixedPose {
        rotation_q29: [i32::MIN; 9],
        translation_q16: [0; 3],
    };
    assert!(!pose.conditioning_warning());
    for row in 0..3 {
        pose.rotation_q29[row * 3 + 1] = i32::MAX;
    }
    assert!(!pose.conditioning_warning());
    for row in 0..3 {
        pose.rotation_q29[row * 3 + 1] = 0;
    }
    assert!(pose.conditioning_warning());
}
