//! Classical and cycle-oriented 1-PPC connected-component reference models.

use std::collections::VecDeque;

/// `(min_x, max_x, min_y, max_y, area)`.
pub type BBox = (usize, usize, usize, usize, usize);

#[inline]
pub fn bit_width(value: usize) -> usize {
    if value <= 1 {
        1
    } else {
        usize::BITS as usize - (value - 1).leading_zeros() as usize
    }
}

#[inline]
pub fn bbox_width(width: usize, height: usize) -> usize {
    2 * bit_width(width) + 2 * bit_width(height)
}

#[inline]
pub fn area_width(width: usize, height: usize) -> usize {
    bit_width(width * height + 1)
}

#[inline]
pub fn feature_width(width: usize, height: usize) -> usize {
    bbox_width(width, height) + area_width(width, height)
}

pub fn moment_zom_width(width: usize, height: usize, pixel_bits: usize) -> usize {
    let max_pixel = (1_u128 << pixel_bits) - 1;
    bit_width_u128(width as u128 * height as u128 * max_pixel + 1)
}

pub fn moment_x_width(width: usize, height: usize, pixel_bits: usize) -> usize {
    let max_pixel = (1_u128 << pixel_bits) - 1;
    let max_x_sum = height as u128 * width as u128 * (width - 1) as u128 / 2;
    bit_width_u128(max_x_sum * max_pixel + 1)
}

pub fn moment_y_width(width: usize, height: usize, pixel_bits: usize) -> usize {
    let max_pixel = (1_u128 << pixel_bits) - 1;
    let max_y_sum = width as u128 * height as u128 * (height - 1) as u128 / 2;
    bit_width_u128(max_y_sum * max_pixel + 1)
}

pub fn moment_feature_width(width: usize, height: usize, pixel_bits: usize) -> usize {
    moment_zom_width(width, height, pixel_bits)
        + moment_x_width(width, height, pixel_bits)
        + moment_y_width(width, height, pixel_bits)
}

#[inline]
fn bit_width_u128(value: u128) -> usize {
    if value <= 1 {
        1
    } else {
        128 - (value - 1).leading_zeros() as usize
    }
}

pub fn encode_moments(
    zom: u128,
    x_fom: u128,
    y_fom: u128,
    width: usize,
    height: usize,
    pixel_bits: usize,
) -> u128 {
    (((y_fom << moment_x_width(width, height, pixel_bits)) | x_fom)
        << moment_zom_width(width, height, pixel_bits))
        | zom
}

pub fn decode_moments(
    mut value: u128,
    width: usize,
    height: usize,
    pixel_bits: usize,
) -> (u128, u128, u128) {
    let z_width = moment_zom_width(width, height, pixel_bits);
    let x_width = moment_x_width(width, height, pixel_bits);
    let zom = value & ((1_u128 << z_width) - 1);
    value >>= z_width;
    let x_fom = value & ((1_u128 << x_width) - 1);
    value >>= x_width;
    (zom, x_fom, value)
}

pub fn encode_bbox(b: BBox, width: usize, height: usize) -> u128 {
    let x_bits = bit_width(width);
    let y_bits = bit_width(height);
    let (min_x, max_x, min_y, max_y, area) = b;
    let mut value = min_x as u128;
    value = (value << x_bits) | max_x as u128;
    value = (value << y_bits) | min_y as u128;
    value = (value << y_bits) | max_y as u128;
    value | ((area as u128) << bbox_width(width, height))
}

pub fn decode_bbox(mut value: u128, width: usize, height: usize) -> BBox {
    let x_bits = bit_width(width);
    let y_bits = bit_width(height);
    let area = (value >> bbox_width(width, height)) as usize;
    let y_mask = (1_u128 << y_bits) - 1;
    let x_mask = (1_u128 << x_bits) - 1;
    let max_y = (value & y_mask) as usize;
    value >>= y_bits;
    let min_y = (value & y_mask) as usize;
    value >>= y_bits;
    let max_x = (value & x_mask) as usize;
    value >>= x_bits;
    let min_x = (value & x_mask) as usize;
    (min_x, max_x, min_y, max_y, area)
}

pub fn empty_bbox(width: usize, height: usize) -> BBox {
    (
        (1 << bit_width(width)) - 1,
        0,
        (1 << bit_width(height)) - 1,
        0,
        0,
    )
}

pub fn include_point(b: BBox, x: usize, y: usize) -> BBox {
    (b.0.min(x), b.1.max(x), b.2.min(y), b.3.max(y), b.4 + 1)
}

pub fn merge_boxes(left: BBox, right: BBox) -> BBox {
    (
        left.0.min(right.0),
        left.1.max(right.1),
        left.2.min(right.2),
        left.3.max(right.3),
        left.4 + right.4,
    )
}

pub fn pad_image(image: &[Vec<u8>]) -> Result<Vec<Vec<u8>>, String> {
    if image.is_empty() {
        return Ok(vec![vec![0, 0], vec![0, 0]]);
    }
    let width = image[0].len() + 2;
    let mut padded = vec![vec![0; width]];
    for row in image {
        if row.len() + 2 != width {
            return Err("all rows must have the same width".into());
        }
        let mut out = Vec::with_capacity(width);
        out.push(0);
        out.extend(row.iter().map(|&pixel| u8::from(pixel != 0)));
        out.push(0);
        padded.push(out);
    }
    padded.push(vec![0; width]);
    Ok(padded)
}

pub fn sort_boxes(boxes: &[BBox]) -> Vec<BBox> {
    let mut result = boxes.to_vec();
    result.sort_unstable();
    result
}

#[derive(Clone, Debug)]
pub struct ClassicalBBoxReference {
    image: Vec<Vec<u8>>,
    pub width: usize,
    pub height: usize,
    pub connectivity: u8,
}

impl ClassicalBBoxReference {
    pub fn new(image: &[Vec<u8>], connectivity: u8) -> Result<Self, String> {
        if !matches!(connectivity, 4 | 8) {
            return Err("connectivity must be 4 or 8".into());
        }
        if image.is_empty() || image[0].is_empty() {
            return Err("image must not be empty".into());
        }
        let width = image[0].len();
        if image.iter().any(|row| row.len() != width) {
            return Err("all rows must have the same width".into());
        }
        Ok(Self {
            image: image
                .iter()
                .map(|row| row.iter().map(|&p| u8::from(p != 0)).collect())
                .collect(),
            width,
            height: image.len(),
            connectivity,
        })
    }

    pub fn boxes(&self) -> Vec<BBox> {
        let mut visited = vec![vec![false; self.width]; self.height];
        let offsets: &[(isize, isize)] = if self.connectivity == 4 {
            &[(-1, 0), (1, 0), (0, -1), (0, 1)]
        } else {
            &[
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1),
                (-1, -1),
                (-1, 1),
                (1, -1),
                (1, 1),
            ]
        };
        let mut boxes = Vec::new();
        for y in 0..self.height {
            for x in 0..self.width {
                if self.image[y][x] == 0 || visited[y][x] {
                    continue;
                }
                let mut queue = VecDeque::from([(x, y)]);
                visited[y][x] = true;
                let mut b = empty_bbox(self.width, self.height);
                while let Some((cx, cy)) = queue.pop_front() {
                    b = include_point(b, cx, cy);
                    for &(dx, dy) in offsets {
                        let nx = cx as isize + dx;
                        let ny = cy as isize + dy;
                        if nx < 0
                            || ny < 0
                            || nx >= self.width as isize
                            || ny >= self.height as isize
                        {
                            continue;
                        }
                        let (nx, ny) = (nx as usize, ny as usize);
                        if self.image[ny][nx] != 0 && !visited[ny][nx] {
                            visited[ny][nx] = true;
                            queue.push_back((nx, ny));
                        }
                    }
                }
                boxes.push(b);
            }
        }
        boxes
    }
}

#[derive(Clone, Debug)]
pub struct RowBufferModel {
    state: Vec<u8>,
}

impl RowBufferModel {
    pub fn new(length: usize) -> Result<Self, String> {
        (length >= 2)
            .then(|| Self {
                state: vec![0; length],
            })
            .ok_or_else(|| "row buffer length must be at least 2".into())
    }
    pub fn out1(&self) -> u8 {
        self.state[self.state.len() - 1]
    }
    pub fn out2(&self) -> u8 {
        self.state[self.state.len() - 2]
    }
    pub fn step(&mut self, datavalid: bool, pix_in: u8) {
        if datavalid {
            self.state.rotate_right(1);
            self.state[0] = pix_in;
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct HolesFillerModel {
    pub top: u8,
    pub left: u8,
    pub x: u8,
    pub right: u8,
}
impl HolesFillerModel {
    pub fn pix_out(&self) -> u8 {
        if self.top != 0 && (self.left != 0 || self.right != 0) {
            1
        } else {
            self.x
        }
    }
    pub fn step(&mut self, datavalid: bool, current: u8, previous: u8) {
        if datavalid {
            let old_x = self.x;
            let old_right = self.right;
            self.top = previous;
            self.left = old_x;
            self.x = old_right;
            self.right = current;
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct WindowModel {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
}
impl WindowModel {
    pub fn outputs(&self) -> (u8, u8, u8, u8) {
        (self.a, self.b, self.c, self.d)
    }
    pub fn step(&mut self, datavalid: bool, current: u8, previous: u8) {
        if datavalid {
            let (old_b, old_d) = (self.b, self.d);
            self.a = old_b;
            self.b = previous;
            self.c = old_d;
            self.d = current;
        }
    }
}

#[derive(Clone, Debug)]
pub struct FeatureAccumulatorModel {
    pub width: usize,
    pub height: usize,
    pub x: usize,
    pub y: usize,
    pub d: BBox,
}
impl FeatureAccumulatorModel {
    pub fn new(width: usize, height: usize, latency: usize) -> Self {
        Self {
            width,
            height,
            x: (width - latency % width) % width,
            y: height - 1,
            d: empty_bbox(width, height),
        }
    }
    pub fn step(
        &mut self,
        datavalid: bool,
        dac: bool,
        dmg: bool,
        clr: bool,
        dp: BBox,
        area_dac: bool,
    ) {
        if !datavalid {
            return;
        }
        let mut current = self.d;
        if dac {
            current = (
                current.0.min(self.x),
                current.1.max(self.x),
                current.2.min(self.y),
                current.3.max(self.y),
                current.4 + usize::from(area_dac),
            );
        }
        if dmg {
            current = merge_boxes(current, dp);
        }
        self.d = if clr {
            empty_bbox(self.width, self.height)
        } else {
            current
        };
        self.x += 1;
        if self.x == self.width {
            self.x = 0;
            self.y = (self.y + 1) % self.height;
        }
    }
}

#[derive(Clone, Debug)]
pub struct TableRamModel<T: Clone> {
    pub ram: Vec<T>,
    read_addr_reg: usize,
}

/// Compatibility spelling retained from the original reference API.
pub type TableRAMModel<T> = TableRamModel<T>;
impl<T: Clone> TableRamModel<T> {
    pub fn new(init: &[T]) -> Self {
        Self {
            ram: init.to_vec(),
            read_addr_reg: 0,
        }
    }
    pub fn prime(&mut self, read_addr: usize) {
        self.read_addr_reg = read_addr;
    }
    pub fn q(&self) -> &T {
        &self.ram[self.read_addr_reg]
    }
    pub fn tick(&mut self, read_addr: usize, write: Option<(usize, T)>) {
        self.read_addr_reg = read_addr;
        if let Some((addr, data)) = write {
            self.ram[addr] = data;
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TableReaderOutputs {
    pub h_raddr: usize,
    pub t_raddr: usize,
    pub n_raddr: usize,
    pub d_raddr: usize,
    pub tp: usize,
    pub dp: u128,
}

#[derive(Clone, Debug)]
pub struct TableReaderModel {
    pub address_bits: usize,
    pub data_width: usize,
    pub table_depth: usize,
    pub p: usize,
    pub hp: usize,
    pub np: usize,
    pub fp: u8,
    pub fn_: u8,
    pc: usize,
    rtp: usize,
    rdp: u128,
}

impl TableReaderModel {
    pub fn new(address_bits: usize, data_width: usize) -> Self {
        Self {
            address_bits,
            data_width,
            table_depth: 1 << address_bits,
            p: 0,
            hp: 0,
            np: 0,
            fp: 0,
            fn_: 0,
            pc: 0,
            rtp: 0,
            rdp: 0,
        }
    }

    pub fn pc(&self) -> usize {
        self.pc
    }

    fn wrapped_inc(&self, value: usize) -> usize {
        (value + 1) % self.table_depth
    }

    #[allow(clippy::too_many_arguments)]
    pub fn compute(
        &self,
        a: u8,
        b: u8,
        h_rdata: usize,
        t_rdata: usize,
        _n_rdata: usize,
        d_rdata: u128,
        h_wdata: usize,
        hcn: bool,
    ) -> TableReaderOutputs {
        let h_raddr = self.pc;
        let n_raddr = self.pc;
        let t_raddr = if hcn { h_wdata } else { h_rdata };
        let d_raddr = if hcn { h_wdata } else { h_rdata };
        let overlap_start = a == 0 && b != 0;
        TableReaderOutputs {
            h_raddr,
            t_raddr,
            n_raddr,
            d_raddr,
            tp: if overlap_start { t_rdata } else { self.rtp },
            dp: if overlap_start { d_rdata } else { self.rdp },
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn tick(
        &mut self,
        datavalid: bool,
        b: u8,
        r1: u8,
        r2: u8,
        d: u128,
        o: bool,
        d_we: bool,
        d_waddr: usize,
        n_rdata: usize,
        h_wdata: usize,
        t_wdata: usize,
        outputs: TableReaderOutputs,
    ) {
        let old_p = self.p;
        let old_hp = self.hp;
        let old_pc = self.pc;
        let dcn = d_we && d_waddr == old_hp;
        let mut next_p = self.p;
        let mut next_hp = self.hp;
        let mut next_np = self.np;
        let mut next_fp = self.fp;
        let mut next_fn = self.fn_;
        let mut next_pc = self.pc;
        let mut next_rtp = self.rtp;
        let mut next_rdp = self.rdp;

        if datavalid {
            next_p = old_pc;
            next_rtp = outputs.tp;
            next_rdp = outputs.dp;
            if r1 != 0 && r2 == 0 {
                next_pc = self.wrapped_inc(old_pc);
            }
            if dcn {
                next_rdp = d;
            }
            if b == 0 && r1 != 0 {
                next_hp = outputs.t_raddr;
                next_fp = u8::from(outputs.t_raddr != old_p);
                next_np = n_rdata;
                next_fn = u8::from(n_rdata == old_p);
            } else if o {
                next_rtp = t_wdata;
                next_fp = 1;
                next_hp = h_wdata;
            }
        }

        self.p = next_p;
        self.hp = next_hp;
        self.np = next_np;
        self.fp = next_fp;
        self.fn_ = next_fn;
        self.pc = next_pc;
        self.rtp = next_rtp;
        self.rdp = next_rdp;
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EquivalenceOutputs {
    pub h_we: u8,
    pub t_we: u8,
    pub n_we: u8,
    pub d_we: u8,
    pub h_waddr: usize,
    pub t_waddr: usize,
    pub n_waddr: usize,
    pub d_waddr: usize,
    pub h_wdata: usize,
    pub t_wdata: usize,
    pub n_wdata: usize,
    pub d_wdata: u128,
    pub hcn: u8,
    pub dac: u8,
    pub dmg: u8,
    pub clr: u8,
    pub eoc: u8,
    pub o: u8,
}

#[derive(Clone, Debug)]
pub struct EquivalenceResolverModel {
    pub address_bits: usize,
    pub table_depth: usize,
    pub cc: usize,
    pub h: usize,
    pub f: u8,
}

impl EquivalenceResolverModel {
    pub fn new(address_bits: usize) -> Self {
        Self {
            address_bits,
            table_depth: 1 << address_bits,
            cc: 0,
            h: 0,
            f: 0,
        }
    }

    fn wrapped_inc(&self, value: usize) -> usize {
        (value + 1) % self.table_depth
    }

    #[allow(clippy::too_many_arguments)]
    pub fn compute(
        &self,
        a: u8,
        b: u8,
        c: u8,
        d: u8,
        p: usize,
        hp: usize,
        np: usize,
        tp: usize,
        dp: u128,
        fp: u8,
        fn_: u8,
        dd: u128,
    ) -> EquivalenceOutputs {
        let ec = c != 0 && d == 0;
        let ep = a != 0 && b == 0;
        let o = b != 0 && d != 0 && (a == 0 || c == 0);
        let mut out = EquivalenceOutputs {
            dmg: u8::from(o && !(self.f != 0 && hp == self.h)),
            dac: u8::from(d != 0),
            clr: u8::from(ec),
            o: u8::from(o),
            ..EquivalenceOutputs::default()
        };
        let mut hbf = false;

        if ec {
            out.n_we = 1;
            out.n_waddr = self.cc;
            out.n_wdata = self.cc;
            out.h_we = 1;
            out.h_waddr = self.cc;
            out.h_wdata = self.cc;
            out.d_we = 1;
            out.d_waddr = if self.f != 0 { self.h } else { self.cc };
            out.d_wdata = dd;
        } else if ep {
            if fp == 0 {
                out.d_we = 1;
                out.d_waddr = np;
                out.d_wdata = dp;
                out.eoc = u8::from(fn_ != 0);
            } else {
                out.h_we = 1;
                out.h_waddr = np;
                out.h_wdata = hp;
                hbf = true;
            }
        } else if o {
            match (self.f != 0, fp != 0) {
                (false, false) => {
                    out.h_we = 1;
                    out.h_waddr = np;
                    out.h_wdata = self.cc;
                    out.t_we = 1;
                    out.t_waddr = out.h_wdata;
                    out.t_wdata = self.cc;
                }
                (false, true) => {
                    out.h_we = 1;
                    out.h_waddr = np;
                    out.h_wdata = hp;
                    out.t_we = 1;
                    out.t_waddr = out.h_wdata;
                    out.t_wdata = self.cc;
                    out.n_we = 1;
                    out.n_waddr = tp;
                    out.n_wdata = self.cc;
                }
                (true, false) => {
                    out.h_we = 1;
                    out.h_waddr = np;
                    out.h_wdata = self.h;
                    out.t_we = 1;
                    out.t_waddr = out.h_wdata;
                    out.t_wdata = self.cc;
                }
                (true, true) => {
                    out.h_we = 1;
                    out.h_waddr = np;
                    out.h_wdata = hp;
                    out.t_we = 1;
                    out.t_waddr = out.h_wdata;
                    out.t_wdata = self.cc;
                    out.n_we = 1;
                    out.n_waddr = tp;
                    out.n_wdata = self.h;
                }
            }
        }
        out.hcn = u8::from(hbf && np == p);
        out
    }

    pub fn tick(&mut self, datavalid: bool, o: u8, h_wdata: usize, clr: u8) {
        if !datavalid {
            return;
        }
        if clr != 0 {
            self.cc = self.wrapped_inc(self.cc);
            self.f = 0;
        } else if o != 0 {
            self.h = h_wdata;
            self.f = 1;
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StreamOutput {
    pub cycle: usize,
    pub bbox: BBox,
}

#[derive(Clone, Debug)]
pub struct LinkRunCcaModel {
    pub width: usize,
    pub height: usize,
    pub max_labels: usize,
    pub label_bits: usize,
    pub table_depth: usize,
    pub cycle: usize,
    pub row_buf_hf: RowBufferModel,
    pub holes_filler: HolesFillerModel,
    pub row_buf_window: RowBufferModel,
    pub window: WindowModel,
    pub feature_accumulator: FeatureAccumulatorModel,
    area_dac_delay: [u8; 3],
    pub table_reader: TableReaderModel,
    pub equivalence_resolver: EquivalenceResolverModel,
    pub next_table: TableRamModel<usize>,
    pub head_table: TableRamModel<usize>,
    pub tail_table: TableRamModel<usize>,
    pub data_table: TableRamModel<u128>,
}

/// Compatibility spelling retained from the original reference API.
pub type LinkRunCCAModel = LinkRunCcaModel;
impl LinkRunCcaModel {
    pub fn new(width: usize, height: usize) -> Result<Self, String> {
        if width < 4 || height < 4 {
            return Err("the streamed image must include a one-pixel border".into());
        }
        let max_labels = width.div_ceil(2).max(1);
        let label_bits = bit_width(max_labels);
        let table_depth = 1 << label_bits;
        Ok(Self {
            width,
            height,
            max_labels,
            label_bits,
            table_depth,
            cycle: 0,
            row_buf_hf: RowBufferModel::new(width - 2)?,
            holes_filler: HolesFillerModel::default(),
            row_buf_window: RowBufferModel::new(width - 2)?,
            window: WindowModel::default(),
            feature_accumulator: FeatureAccumulatorModel::new(width, height, 3),
            area_dac_delay: [0; 3],
            table_reader: TableReaderModel::new(label_bits, 0),
            equivalence_resolver: EquivalenceResolverModel::new(label_bits),
            next_table: TableRamModel::new(&(0..table_depth).collect::<Vec<_>>()),
            head_table: TableRamModel::new(&(0..table_depth).collect::<Vec<_>>()),
            tail_table: TableRamModel::new(&(0..table_depth).collect::<Vec<_>>()),
            data_table: TableRamModel::new(&vec![
                encode_bbox(
                    empty_bbox(width, height),
                    width,
                    height
                );
                table_depth
            ]),
        })
    }

    pub fn step(&mut self, pixel: u8, datavalid: bool) -> Vec<StreamOutput> {
        let left = self.holes_filler.left;
        let hr1 = self.row_buf_hf.out1();
        let hf_out = self.holes_filler.pix_out();
        let r1 = self.row_buf_window.out1();
        let r2 = self.row_buf_window.out2();
        let (a, b, c, d) = self.window.outputs();
        let current_bbox_packed = encode_bbox(self.feature_accumulator.d, self.width, self.height);
        let area_dac = self.area_dac_delay[2];

        let preview_hcn = a != 0
            && b == 0
            && self.table_reader.fp != 0
            && self.table_reader.np == self.table_reader.p;
        let table_outputs = self.table_reader.compute(
            a,
            b,
            *self.head_table.q(),
            *self.tail_table.q(),
            *self.next_table.q(),
            *self.data_table.q(),
            self.table_reader.hp,
            preview_hcn,
        );
        let equivalence_outputs = self.equivalence_resolver.compute(
            a,
            b,
            c,
            d,
            self.table_reader.p,
            self.table_reader.hp,
            self.table_reader.np,
            table_outputs.tp,
            table_outputs.dp,
            self.table_reader.fp,
            self.table_reader.fn_,
            current_bbox_packed,
        );

        let outputs = if datavalid && equivalence_outputs.eoc != 0 {
            vec![StreamOutput {
                cycle: self.cycle,
                bbox: decode_bbox(table_outputs.dp, self.width, self.height),
            }]
        } else {
            Vec::new()
        };

        self.feature_accumulator.step(
            datavalid,
            equivalence_outputs.dac != 0,
            equivalence_outputs.dmg != 0,
            equivalence_outputs.clr != 0,
            decode_bbox(table_outputs.dp, self.width, self.height),
            area_dac != 0,
        );
        self.table_reader.tick(
            datavalid,
            b,
            r1,
            r2,
            current_bbox_packed,
            equivalence_outputs.o != 0,
            equivalence_outputs.d_we != 0,
            equivalence_outputs.d_waddr,
            *self.next_table.q(),
            equivalence_outputs.h_wdata,
            equivalence_outputs.t_wdata,
            table_outputs,
        );
        self.equivalence_resolver.tick(
            datavalid,
            equivalence_outputs.o,
            equivalence_outputs.h_wdata,
            equivalence_outputs.clr,
        );
        self.row_buf_hf.step(datavalid, left);
        self.holes_filler.step(datavalid, u8::from(pixel != 0), hr1);
        self.row_buf_window.step(datavalid, c);
        self.window.step(datavalid, hf_out, r1);
        if datavalid {
            self.area_dac_delay = [
                u8::from(pixel != 0),
                self.area_dac_delay[0],
                self.area_dac_delay[1],
            ];
        }
        self.next_table.tick(
            table_outputs.n_raddr,
            (datavalid && equivalence_outputs.n_we != 0)
                .then_some((equivalence_outputs.n_waddr, equivalence_outputs.n_wdata)),
        );
        self.head_table.tick(
            table_outputs.h_raddr,
            (datavalid && equivalence_outputs.h_we != 0)
                .then_some((equivalence_outputs.h_waddr, equivalence_outputs.h_wdata)),
        );
        self.tail_table.tick(
            table_outputs.t_raddr,
            (datavalid && equivalence_outputs.t_we != 0)
                .then_some((equivalence_outputs.t_waddr, equivalence_outputs.t_wdata)),
        );
        self.data_table.tick(
            table_outputs.d_raddr,
            (datavalid && equivalence_outputs.d_we != 0)
                .then_some((equivalence_outputs.d_waddr, equivalence_outputs.d_wdata)),
        );
        self.cycle += 1;
        outputs
    }

    pub fn process(&mut self, image: &[Vec<u8>]) -> Result<Vec<StreamOutput>, String> {
        if image.len() != self.height || image.iter().any(|r| r.len() != self.width) {
            return Err("image dimensions do not match model dimensions".into());
        }
        let mut outputs = Vec::new();
        for row in image {
            for &pixel in row {
                outputs.extend(self.step(pixel, true));
            }
        }
        for _ in 0..4 {
            outputs.extend(self.step(0, true));
        }
        Ok(outputs)
    }
    pub fn boxes(&mut self, image: &[Vec<u8>]) -> Result<Vec<BBox>, String> {
        Ok(self.process(image)?.into_iter().map(|o| o.bbox).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn widths_and_round_trips() {
        assert_eq!(bit_width(1), 1);
        assert_eq!(bit_width(48), 6);
        let b = (1, 31, 2, 17, 93);
        assert_eq!(decode_bbox(encode_bbox(b, 48, 36), 48, 36), b);
        let moments = (12345, 54321, 98765);
        assert_eq!(
            decode_moments(
                encode_moments(moments.0, moments.1, moments.2, 1280, 800, 8),
                1280,
                800,
                8
            ),
            moments
        );
    }

    #[test]
    fn four_and_eight_connectivity_differ_on_diagonal() {
        let image = vec![vec![1, 0], vec![0, 1]];
        assert_eq!(
            ClassicalBBoxReference::new(&image, 4)
                .unwrap()
                .boxes()
                .len(),
            2
        );
        assert_eq!(
            ClassicalBBoxReference::new(&image, 8).unwrap().boxes(),
            vec![(0, 1, 0, 1, 2)]
        );
    }

    #[test]
    fn primitive_pipeline_models_hold_on_invalid_cycles() {
        let mut row = RowBufferModel::new(3).unwrap();
        row.step(true, 4);
        row.step(true, 5);
        row.step(false, 9);
        row.step(true, 6);
        assert_eq!((row.out1(), row.out2()), (4, 5));
        let hole = HolesFillerModel {
            top: 1,
            left: 1,
            ..HolesFillerModel::default()
        };
        assert_eq!(hole.pix_out(), 1);
    }

    #[test]
    fn table_reader_matches_frozen_register_transition() {
        let mut reader = TableReaderModel::new(2, 0);
        let outputs = reader.compute(0, 1, 2, 3, 1, 99, 0, false);
        assert_eq!(
            outputs,
            TableReaderOutputs {
                h_raddr: 0,
                t_raddr: 2,
                n_raddr: 0,
                d_raddr: 2,
                tp: 3,
                dp: 99,
            }
        );
        reader.tick(true, 0, 1, 0, 55, false, false, 0, 1, 2, 3, outputs);
        assert_eq!(
            (
                reader.p,
                reader.hp,
                reader.np,
                reader.fp,
                reader.fn_,
                reader.pc()
            ),
            (0, 2, 1, 1, 0, 1)
        );

        let held = reader.clone();
        reader.tick(
            false,
            1,
            1,
            0,
            123,
            true,
            true,
            2,
            3,
            1,
            0,
            TableReaderOutputs::default(),
        );
        assert_eq!(
            (
                reader.p,
                reader.hp,
                reader.np,
                reader.fp,
                reader.fn_,
                reader.pc()
            ),
            (held.p, held.hp, held.np, held.fp, held.fn_, held.pc())
        );
    }

    #[test]
    fn equivalence_resolver_covers_create_retire_and_overlap() {
        let mut resolver = EquivalenceResolverModel::new(3);
        let create = resolver.compute(0, 0, 1, 0, 1, 2, 3, 4, 77, 1, 1, 88);
        assert_eq!(
            (
                create.h_we,
                create.n_we,
                create.d_we,
                create.d_wdata,
                create.clr
            ),
            (1, 1, 1, 88, 1)
        );
        resolver.tick(true, create.o, create.h_wdata, create.clr);
        assert_eq!((resolver.cc, resolver.f), (1, 0));

        let retire = resolver.compute(1, 0, 0, 0, 1, 2, 3, 4, 77, 0, 1, 88);
        assert_eq!(
            (retire.d_we, retire.d_waddr, retire.d_wdata, retire.eoc),
            (1, 3, 77, 1)
        );

        resolver.f = 1;
        resolver.h = 2;
        let overlap = resolver.compute(0, 1, 0, 1, 1, 2, 3, 4, 77, 1, 1, 88);
        assert_eq!(
            (
                overlap.h_we,
                overlap.t_we,
                overlap.n_we,
                overlap.h_wdata,
                overlap.n_wdata,
                overlap.dmg,
                overlap.o
            ),
            (1, 1, 1, 2, 2, 0, 1)
        );
    }

    fn framed_image(width: usize, height: usize, points: &[(usize, usize)]) -> Vec<Vec<u8>> {
        let mut image = vec![vec![0; width]; height];
        for &(x, y) in points {
            image[y][x] = 1;
        }
        image
    }

    #[test]
    fn link_run_model_has_frozen_cycle_outputs() {
        let cases = [
            (vec![(2, 2)], vec![(30, (2, 2, 2, 2, 1))]),
            (
                vec![(1, 1), (5, 3)],
                vec![(21, (1, 1, 1, 1, 1)), (41, (5, 5, 3, 3, 1))],
            ),
            (vec![(2, 2), (3, 3)], vec![(39, (2, 3, 2, 3, 2))]),
            (
                vec![(2, 2), (3, 2), (2, 3), (3, 3)],
                vec![(39, (2, 3, 2, 3, 4))],
            ),
        ];
        for (points, expected) in cases {
            let mut model = LinkRunCcaModel::new(8, 6).unwrap();
            let actual = model
                .process(&framed_image(8, 6, &points))
                .unwrap()
                .into_iter()
                .map(|output| (output.cycle, output.bbox))
                .collect::<Vec<_>>();
            assert_eq!(actual, expected);
        }
    }

    #[test]
    fn link_run_invalid_cycles_hold_pipeline_state() {
        let image = framed_image(8, 6, &[(2, 2)]);
        let mut model = LinkRunCcaModel::new(8, 6).unwrap();
        let mut outputs = Vec::new();
        for (index, pixel) in image.into_iter().flatten().enumerate() {
            if matches!(index, 10 | 20 | 21) {
                outputs.extend(model.step(0, false));
            }
            outputs.extend(model.step(pixel, true));
        }
        for _ in 0..4 {
            outputs.extend(model.step(0, true));
        }
        assert_eq!(model.cycle, 55);
        assert_eq!(
            outputs,
            vec![StreamOutput {
                cycle: 33,
                bbox: (2, 2, 2, 2, 1)
            }]
        );
    }

    #[test]
    fn link_run_matches_classical_eight_connectivity() {
        let (width, height) = (16, 10);
        let mut state = 0x8c0c_u64;
        for _case in 0..64 {
            let mut image = vec![vec![0; width]; height];
            for row in image.iter_mut().take(height - 1).skip(1) {
                for pixel in row.iter_mut().take(width - 1).skip(1) {
                    state ^= state << 13;
                    state ^= state >> 7;
                    state ^= state << 17;
                    *pixel = u8::from(state.is_multiple_of(5));
                }
            }
            let expected = sort_boxes(&ClassicalBBoxReference::new(&image, 8).unwrap().boxes());
            let mut model = LinkRunCcaModel::new(width, height).unwrap();
            assert_eq!(sort_boxes(&model.boxes(&image).unwrap()), expected);
        }
    }

    #[test]
    fn link_run_matches_directed_chain_merge() {
        let image = vec![
            vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            vec![0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
            vec![0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
            vec![0, 0, 1, 0, 1, 1, 0, 1, 0, 0],
            vec![0, 0, 1, 0, 1, 0, 0, 1, 0, 0],
            vec![0, 0, 1, 0, 1, 0, 1, 1, 0, 0],
            vec![0, 0, 1, 0, 1, 0, 0, 1, 0, 0],
            vec![0, 0, 1, 0, 1, 1, 1, 1, 0, 0],
            vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ];
        let expected = sort_boxes(&ClassicalBBoxReference::new(&image, 8).unwrap().boxes());
        let mut model = LinkRunCcaModel::new(10, 10).unwrap();
        assert_eq!(sort_boxes(&model.boxes(&image).unwrap()), expected);
    }

    #[test]
    fn link_run_exhausts_all_three_by_three_interiors() {
        for bits in 0_u16..(1 << 9) {
            let mut image = vec![vec![0; 5]; 5];
            for index in 0..9 {
                image[1 + index / 3][1 + index % 3] = ((bits >> index) & 1) as u8;
            }
            let expected = sort_boxes(&ClassicalBBoxReference::new(&image, 8).unwrap().boxes());
            let mut model = LinkRunCcaModel::new(5, 5).unwrap();
            assert_eq!(
                sort_boxes(&model.boxes(&image).unwrap()),
                expected,
                "interior bits={bits:09b}"
            );
        }
    }
}
