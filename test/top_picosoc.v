`define MPRJ_IO_PADS 38

module picosoc_top #(
    parameter BITS = 32
)(
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,
);
    wire clk = io_in[0];
    wire rst = io_in[1];

    wire        iomem_valid;
    reg         iomem_ready;
    wire [3:0]  iomem_wstrb;
    wire [31:0] iomem_addr;
    wire [31:0] iomem_wdata;
    reg  [31:0] iomem_rdata;

    reg [31:0] gpio;

    always @(posedge clk) begin
        if (rst) begin
            gpio <= 0;
        end else begin
            iomem_ready <= 0;
            if (iomem_valid && !iomem_ready && iomem_addr[31:24] == 8'h 03) begin
                iomem_ready <= 1;
                iomem_rdata <= gpio;
                if (iomem_wstrb[0]) gpio[ 7: 0] <= iomem_wdata[ 7: 0];
                if (iomem_wstrb[1]) gpio[15: 8] <= iomem_wdata[15: 8];
                if (iomem_wstrb[2]) gpio[23:16] <= iomem_wdata[23:16];
                if (iomem_wstrb[3]) gpio[31:24] <= iomem_wdata[31:24];
            end
        end
    end

    wire [3:0] flash_oe;

    picosoc #(
        .MEM_WORDS    (128)
    ) soc (
        .clk          (clk),
        .resetn       (!rst),

        .ser_tx       (io_out[2]),
        .ser_rx       (io_in[3]),

        .flash_csb    (io_out[4]),
        .flash_clk    (io_out[5]),

        .flash_io0_oe (flash_oe[0]),
        .flash_io1_oe (flash_oe[1]),
        .flash_io2_oe (flash_oe[2]),
        .flash_io3_oe (flash_oe[3]),

        .flash_io0_do (io_out[6]),
        .flash_io1_do (io_out[7]),
        .flash_io2_do (io_out[8]),
        .flash_io3_do (io_out[9]),

        .flash_io0_di (io_in[6]),
        .flash_io1_di (io_in[7]),
        .flash_io2_di (io_in[8]),
        .flash_io3_di (io_in[9]),

        .irq_5        (1'b0        ),
        .irq_6        (1'b0        ),
        .irq_7        (1'b0        ),

        .iomem_valid  (iomem_valid ),
        .iomem_ready  (iomem_ready ),
        .iomem_wstrb  (iomem_wstrb ),
        .iomem_addr   (iomem_addr  ),
        .iomem_wdata  (iomem_wdata ),
        .iomem_rdata  (iomem_rdata )
    );

    assign io_out[1:0] = 2'b00;
    assign io_out[3] = 1'b0;
    assign io_oeb[5:0] = 2'b001011;
    assign io_oeb[9:6] = ~flash_oe;

    assign io_oeb[`MPRJ_IO_PADS-1:10] = 32'h0;
    assign io_out[`MPRJ_IO_PADS-1:10] = gpio;
endmodule
