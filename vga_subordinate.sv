module vga_subordinate #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    parameter logic [ADDR_WIDTH-1:0] BASE_ADDR_1 = 'h8000_0000,
    parameter logic [ADDR_WIDTH-1:0] BASE_ADDR_2 = 'h8000_0000
)(
    input CLK, nRST,
    input logic [DATA_WIDTH-1:0] fb_read,
    output logic buffer_select, // Double Buffer
    output logic fb_ren,
    output logic fb_wen,
    output logic [ADDR_WIDTH-1:0] fb_waddr,
    output logic [ADDR_WIDTH-1:0] fb_raddr,
    bus_protocol_if.peripheral_vital busif
);
    localparam int BUFFER_SIZE = 230400; // 240 x 480

    logic next_fb_ren, next_fb_wen, next_buffer_select;
    logic [ADDR_WIDTH-1:0] next_fb_raddr;
    logic [ADDR_WIDTH-1:0] next_fb_waddr;
    logic [DATA_WIDTH-1:0] next_rdata;
    always_comb begin
        next_fb_raddr = busif.addr;
        next_fb_ren = busif.ren;
        next_fb_wen = busif.wen;
        next_rdata = DATA_WIDTH'b0;
        next_fb_waddr = fb_waddr;
        next_buffer_select = buffer_select;

        if(busif.wen) begin
            next_rdata = busif.wdata;
            next_fb_waddr = fb_waddr + 4;
            if(fb_waddr == (BASE_ADDR_1 + BUFFER_SIZE - 4) && buffer_select == 1'b0) begin
                next_buffer_select = 1'b1;
                next_fb_waddr = BASE_ADDR_2;
            end
            else if(fb_waddr == (BASE_ADDR_2 + BUFFER_SIZE - 4) && buffer_select == 1'b1) begin
                next_buffer_select = 1'b0;
                next_fb_waddr = BASE_ADDR_1;
            end
        end
        else if (busif.ren) begin
            next_rdata = fb_read;
        end
    end

    always_ff @(posedge CLK, negedge nRST) begin
        if(!nRST) begin
            fb_ren <= 1'b0;
            fb_wen <= 1'b0;
            fb_raddr <= ADDR_WIDTH'b0;
            fb_waddr <= BASE_ADDR_1;
            busif.rdata <= DATA_WIDTH'b0;
            busif.error <= 1'b0;
            buffer_select <= 1'b0;
        end
        else begin
            fb_ren <= next_fb_ren;
            fb_wen <= next_fb_wen;
            fb_raddr <= next_fb_raddr;
            fb_waddr <= next_fb_waddr;
            busif.rdata <= next_rdata;
            busif.error <= 1'b0;
            buffer_select <= next_buffer_select;
        end
    end

    always_comb begin
        busif.request_stall = busif.wen || busif.ren;
    end

endmodule
