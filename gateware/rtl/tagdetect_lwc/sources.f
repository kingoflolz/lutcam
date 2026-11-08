# Complete production closure for the combined TagDetect, LFTag, and LWC pipeline.
# Paths are relative to the repository root and ordered by dependency.

# LRCCA / TagDetect
rtl/lrcca/lrcca_util_pkg.sv
rtl/lrcca/TwoPPCS0Stage.sv
rtl/lrcca/TwoPPCS1Stage.sv
rtl/lrcca/TwoPPCS2Stage.sv
rtl/lrcca/TwoPPCS6BStage.sv
rtl/lrcca/TwoPPCS3Stage.sv
rtl/lrcca/TwoPPCS4Stage.sv
rtl/lrcca/S5HighChunkAdd.sv
rtl/lrcca/TwoPPCS5Stage.sv
rtl/lrcca/TwoPPC.sv
rtl/lrcca/CentroidMantissaWindowStage.sv
rtl/lrcca/CentroidMantissaWindow.sv
rtl/lrcca/CentroidSignedWindowStage.sv
rtl/lrcca/CentroidSignedMomentNormalizer.sv
rtl/lrcca/CentroidPower2MultiplierNormalizer.sv
rtl/lrcca/CentroidMidpointReflectionReciprocal.sv
rtl/lrcca/CentroidCompressor.sv
rtl/lrcca/ChildArenaSyncFifo.sv
rtl/lrcca/ChildArenaBundleFifo.sv
rtl/lrcca/ChildArenaReturnedIdFifo.sv
rtl/lrcca/ChildArenaFvMem.sv
rtl/lrcca/ChildArena.sv
rtl/lrcca/HierarchicalTwoPPC.sv
rtl/lrcca/Ll4CausalMean5x3.sv
rtl/lrcca/ThresholdMomentWeightRom.sv
rtl/lrcca/ThresholdConnectivity2PPC.sv
rtl/lrcca/AdaptiveThresholdFrontend.sv
rtl/lrcca/Ll2CausalBoxReference.sv
rtl/lrcca/WaveletTagDetect.sv

# LWC encoder
rtl/lwc/codec_profile_pkg.sv
rtl/lwc/LwcRegisteredAddSub.sv
rtl/lwc/horizontal_lift_stage.sv
rtl/lwc/horizontal_lift_row_service.sv
rtl/lwc/vertical_lift_scheduler.sv
rtl/lwc/first_horizontal_lift.sv
rtl/lwc/first_vertical_lift.sv
rtl/lwc/forward_wavelet_transform.sv
rtl/lwc/unified_wavelet_frontend.sv
rtl/lwc/coefficient_quantizer.sv
rtl/lwc/coefficient_pair_queues.sv
rtl/lwc/LwcEncoderAccumulator.sv
rtl/lwc/causal_group_encoder.sv

# Standalone LFTag sparse decoder/localizer
rtl/lftag/lftag_pkg.sv
rtl/lftag/LfTagCandidateFifoRamMetadata.sv
rtl/lftag/LfTagCornerSelect9.sv
rtl/lftag/LfTagHomogeneousGeometryScorer.sv
rtl/lftag/LfTagReciprocalSqrt512.sv
rtl/lftag/LfTagInitialHomographyCalibrationDatapath.sv
rtl/lftag/LfTagInitialHomographyNormalizeDatapath.sv
rtl/lftag/LfTagInitialHomographyFused.sv
rtl/lftag/LfTagPoseReconstruct.sv
rtl/lftag/LfTagCompact3x3DecoderLocalizer.sv
rtl/lftag/SharedLfTagDecoder.sv

rtl/tagdetect_lwc/TagDetectLwc.sv
