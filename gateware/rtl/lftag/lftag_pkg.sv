package lftag_pkg;
    typedef enum logic [4:0] {
        REJECT_NONE              = 5'd0,
        REJECT_INVALID_CENTROID  = 5'd1,
        REJECT_NONFINITE         = 5'd2,
        // Wire value 3 is reserved.
        REJECT_MASS_SPREAD       = 5'd4,
        REJECT_BASELINE_SHORT    = 5'd5,
        REJECT_DOMINANT_SIDE     = 5'd6,
        REJECT_CORNER_SELECTION  = 5'd7,
        REJECT_INVALID_QUAD      = 5'd8,
        REJECT_COLLINEAR         = 5'd9,
        REJECT_SINGULAR_MAP      = 5'd10,
        REJECT_NO_HYPOTHESIS     = 5'd11,
        REJECT_RESIDUAL          = 5'd12,
        // Wire value 13 is reserved.
        REJECT_PROTOCOL          = 5'd14,
        REJECT_WATCHDOG          = 5'd15
    } lftag_reject_t;

endpackage
