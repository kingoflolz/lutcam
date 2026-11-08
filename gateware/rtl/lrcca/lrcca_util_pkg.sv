package lrcca_util_pkg;
  function automatic int bit_width(input longint unsigned value);
    longint unsigned cursor;
    begin
      cursor = (value == 0) ? 0 : value - 1;
      bit_width = 1;
      while ((cursor >> bit_width) != 0) begin
        bit_width++;
      end
    end
  endfunction

  function automatic int max_int(input int left, input int right);
    begin
      max_int = (left > right) ? left : right;
    end
  endfunction

  function automatic int min_int(input int left, input int right);
    begin
      min_int = (left < right) ? left : right;
    end
  endfunction

  function automatic int area_width(input int width, input int height);
    begin
      area_width = bit_width(longint'(width) * longint'(height) + 1);
    end
  endfunction

  function automatic int moment_zom_width(
    input int width,
    input int height,
    input int pixel_bits
  );
    longint unsigned max_pixel;
    begin
      max_pixel = (longint'(1) << pixel_bits) - 1;
      moment_zom_width =
          bit_width(longint'(width) * longint'(height) * max_pixel + 1);
    end
  endfunction

  function automatic int moment_x_width(
    input int width,
    input int height,
    input int pixel_bits
  );
    longint unsigned max_pixel;
    longint unsigned max_x_sum;
    begin
      max_pixel = (longint'(1) << pixel_bits) - 1;
      max_x_sum = longint'(height) * longint'(width) * (longint'(width) - 1) / 2;
      moment_x_width = bit_width(max_x_sum * max_pixel + 1);
    end
  endfunction

  function automatic int moment_y_width(
    input int width,
    input int height,
    input int pixel_bits
  );
    longint unsigned max_pixel;
    longint unsigned max_y_sum;
    begin
      max_pixel = (longint'(1) << pixel_bits) - 1;
      max_y_sum = longint'(width) * longint'(height) * (longint'(height) - 1) / 2;
      moment_y_width = bit_width(max_y_sum * max_pixel + 1);
    end
  endfunction

  function automatic int centered_moment_bits(
    input int length,
    input int orthogonal_length,
    input int pixel_bits
  );
    longint unsigned max_pixel;
    longint unsigned sum_abs;
    longint signed centered;
    begin
      max_pixel = (64'd1 << pixel_bits) - 1;
      sum_abs = 0;
      for (int coordinate = 0; coordinate < length; coordinate++) begin
        centered = (longint'(2) * longint'(coordinate)) - (longint'(length) - 1);
        sum_abs += (centered < 0) ? -centered : centered;
      end
      centered_moment_bits =
          bit_width(((longint'(orthogonal_length) * max_pixel * sum_abs) / 2) + 1) + 1;
    end
  endfunction
endpackage
