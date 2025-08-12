//! @file xSPI.v
//! @brief Top-level and submodules for a simple SPI-like protocol (xSPI) with controller and slave.

/**
 * @module xspi_top
 * @brief Top-level module connecting xSPI controller and slave via a shared IO bus.
 * @param clk     System clock
 * @param rst_n   Active-low reset
 * @param start   Start transaction
 * @param command 8-bit command
 * @param address 48-bit address
 * @param wr_data 64-bit data to write
 * @param rd_data 64-bit data read
 * @param done    Transaction done flag
 * @param ready   Slave ready flag
 */
module xspi_top (
    input  wire        clk,      //!< System clock
    input  wire        rst_n,    //!< Active-low reset
    input  wire        start,    //!< Start transaction
    //input  wire        rw,
    input  wire [7:0]  command,  //!< 8-bit command
    input  wire [47:0] address,  //!< 48-bit address
    input  wire [63:0] wr_data,  //!< 64-bit data to write
    output wire [63:0] rd_data,  //!< 64-bit data read
    output wire        done,     //!< Transaction done flag
    output wire        ready,    //!< Slave ready flag
    output wire        crc_ca_match_slave,
    output wire        crc_ca_error_slave,
    output wire        crc_ca_match_master,
    output wire        crc_ca_error_master,
    output wire        crc_data_match_master,
    output wire        crc_data_error_master,
    output wire        crc_data_match_slave,
    output wire        crc_data_error_slave
);

    // Internal bus and control signals
    wire        cs_n;           //!< Chip select (active low)
    wire        sck;            //!< SPI clock
    wire [7:0]  master_io_out;  //!< Controller output to bus
    wire [7:0]  slave_io_out;   //!< Slave output to bus
    wire        master_io_oe;   //!< Controller output enable
    wire        slave_io_oe;    //!< Slave output enable
    wire [7:0]  io_bus;         //!< Shared IO bus
    // CRC signals
   // wire        crc_ca_match, crc_data_match, crc_ca_error, crc_data_error;

    // Simplified IO bus - no tri-state logic
    assign io_bus = master_io_oe ? master_io_out : 
                    slave_io_oe  ? slave_io_out  : 8'h00;

    /**
     * @brief xSPI controller (master)
     */
    xspi_sopi_controller master (
        .clk(clk),
        .rst_n(rst_n),
        .cs_n(cs_n),
        .sck(sck),
        .io_out(master_io_out),
        .io_in(io_bus),
        .io_oe(master_io_oe),
        .start(start),
        //.rw(rw),
        .command_in(command),
        .address_in(address),
        .wr_data_in(wr_data),
        .rd_data(rd_data),
        .done(done),
        .data_strobe(data_strobe),
        .crc_ca_match(crc_ca_match_master),
        .crc_data_match(crc_data_match_master),
        .crc_ca_error(crc_ca_error_master),
        .crc_data_error(crc_data_error_master),
        .crc_ca_error_slave(crc_ca_error_slave),
        .crc_data_error_slave(crc_data_error_slave)
    );

    /**
     * @brief xSPI slave
     */
    xspi_sopi_slave slave (
        .clk(clk),
        .rst_n(rst_n),
        .cs_n(cs_n),
        .sck(sck),
        .io_out(slave_io_out),
        .io_in(io_bus),
        .io_oe(slave_io_oe),
        .ready(ready),
        .data_strobe(data_strobe),
        .crc_ca_match(crc_ca_match_slave),
        .crc_data_match(crc_data_match_slave),
        .crc_ca_error(crc_ca_error_slave),
        .crc_data_error(crc_data_error_slave),
        .crc_ca_error_master(crc_ca_error_master),
        .crc_data_error_master(crc_data_error_master)
    );


endmodule

/**
 * @module xspi_sopi_controller
 * @brief xSPI controller (master) for command/address/data transfer.
 * @param clk      System clock
 * @param rst_n    Active-low reset
 * @param cs_n     Chip select (active low)
 * @param sck      SPI clock
 * @param io_out   Output to IO bus
 * @param io_in    Input from IO bus
 * @param io_oe    Output enable for IO bus
 * @param start    Start transaction
 * @param command  8-bit command
 * @param address  48-bit address
 * @param wr_data  64-bit data to write
 * @param rd_data  64-bit data read
 * @param done     Transaction done flag
 */
module xspi_sopi_controller (
    input  wire        clk,      //!< System clock
    input  wire        rst_n,    //!< Active-low reset

    // SPI signals - separated input/output
    output reg         cs_n,     //!< Chip select (active low)
    output reg         sck,      //!< SPI clock
    output reg [7:0]   io_out,   //!< Output to IO bus
    input  wire [7:0]  io_in,    //!< Input from IO bus
    output reg         io_oe,    //!< Output enable for IO bus

    // Control signals
    input  wire        start,    //!< Start transaction
   // input  wire        rw,         // 0 = write, 1 = read
  input  wire [7:0]  command_in,  //!< 8-bit command
  input  wire [47:0] address_in,  //!< 48-bit address
  input  wire [63:0] wr_data_in,  //!< 64-bit data to write
    input wire         data_strobe,

    output reg [63:0]  rd_data,  //!< 64-bit data read
    output reg         done,     //!< Transaction done flag
    output reg         crc_ca_match,
    output reg         crc_data_match,
    output reg         crc_ca_error,
    output reg         crc_data_error,
    input wire         crc_ca_error_slave,
    input wire         crc_data_error_slave
);
    reg [2:0] wait_cycles;
    reg [7:0] command;
    reg [47:0] address;
    reg [63:0] wr_data;
    reg [3:0] retransmit_cnt;  // Retransmission counter
    // === FSM States ===
    reg [3:0] state;         //!< Current FSM state
    reg [3:0] next_state;    //!< Next FSM state
    localparam Latency         = 3'd6;
    localparam STATE_IDLE      = 4'd0; //!< Idle state
    localparam STATE_CMD       = 4'd1; //!< Send command
    localparam STATE_ADDR      = 4'd2; //!< Send address
    localparam STATE_SEND_CRC_CA = 4'd3; //!< Send CRC for command/address
    localparam STATE_WR_DATA   = 4'd4; //!< Write data
    localparam STATE_SEND_CRC_DATA = 4'd5; //!< Send CRC for data
    localparam STATE_RD_DATA   = 4'd6; //!< Read data
    localparam STATE_RECV_CRC_DATA = 4'd7; //!< Receive CRC for data
    localparam STATE_RECV_CRC_CA = 4'd8; //!< Receive CRC for command/address
    localparam STATE_FINISH    = 4'd9; //!< Finish/cleanup
    // === Counters and Buffers ===
    reg [3:0] byte_cnt;      //!< Byte counter for address/data
    reg [63:0] rdata_buf;    //!< Buffer for read data
    // CRC for command/address
    reg crc_ca_clear, crc_ca_enable;
    wire [7:0] crc_ca_out;
    reg [7:0] crc_ca_recv;
    // CRC for data
    reg crc_data_clear, crc_data_enable;
    wire [7:0] crc_data_out;
    reg [7:0] crc_data_recv;
    reg [7:0] data_for_crc;
    // CRC8 for command/address
    crc8 crc_ca_inst (
        .clk(clk),
        .rst(!rst_n),
        .enable(crc_ca_enable),
        .clear(crc_ca_clear),
        .data_in(data_for_crc),
        .crc_out(crc_ca_out)
    );
    // CRC8 for data
    crc8 crc_data_inst (
        .clk(clk),
        .rst(!rst_n),
        .enable(crc_data_enable),
        .clear(crc_data_clear),
        .data_in(data_for_crc),
        .crc_out(crc_data_out)
    );

    /**
     * @brief FSM sequential logic: state update
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= STATE_IDLE;
        else
            state <= next_state;
    end

    /**
     * @brief FSM combinational logic: next state logic
     */
    always @(*) begin
        next_state = state;
        case (state)
            STATE_IDLE:
                if (start)
                    next_state = STATE_CMD;
            STATE_CMD:
                next_state = STATE_ADDR;
            STATE_ADDR:
                if (byte_cnt == 6)
                    next_state = STATE_SEND_CRC_CA;
            STATE_SEND_CRC_CA:
                next_state = (command == 8'hFF) ? STATE_RECV_CRC_CA : (command == 8'hA5) ? STATE_WR_DATA : STATE_FINISH;
            STATE_RECV_CRC_CA:
                next_state = (command == 8'hFF) ? STATE_RD_DATA : STATE_FINISH;
            STATE_WR_DATA:
                if (byte_cnt == 14)
                    next_state = STATE_SEND_CRC_DATA;
            STATE_SEND_CRC_DATA:
                next_state = STATE_FINISH;
            STATE_RD_DATA:
                if (byte_cnt == 15)
                    next_state = STATE_RECV_CRC_DATA;
            STATE_RECV_CRC_DATA:
                next_state = STATE_FINISH;
            STATE_FINISH:
                // Check for retransmission conditions
                if ((crc_ca_error_slave == 1'b1 || crc_data_error_slave == 1'b1 || crc_data_error == 1'b1) && retransmit_cnt < 4'd3) begin
                    next_state = STATE_CMD;  // Retransmit from command
                end else begin
                    next_state = STATE_IDLE;
                end
            default:
                next_state = STATE_IDLE;
        endcase
    end

    /**
     * @brief Main operation: drive SPI signals and manage data transfer
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_n     <= 1'b1;
            sck      <= 1'b0;
            io_out   <= 8'h00;
            io_oe    <= 1'b0;
            done     <= 1'b0;
            rd_data  <= 64'h0;
            rdata_buf <= 64'h0;
            byte_cnt <= 4'd0;
            wait_cycles <= 3'd0;
            crc_ca_clear <= 1'b1;
            crc_ca_enable <= 1'b0;
            crc_data_clear <= 1'b1;
            crc_data_enable <= 1'b0;
            crc_ca_match <= 1'b0;
            crc_ca_error <= 1'b0;
            crc_data_match <= 1'b0;
            crc_data_error <= 1'b0;
            retransmit_cnt <= 0;

        end else begin
            // Default disables
            crc_ca_enable <= 1'b0;
            crc_data_enable <= 1'b0;
            case (state)
                STATE_IDLE: begin
                    done     <= 1'b0;
                    cs_n     <= 1'b1;
                    sck      <= 1'b0;
                    io_oe    <= 1'b0;
                    byte_cnt <= 4'd0;
                    wait_cycles <= 0;
                    crc_ca_clear <= 1'b1;
                    crc_data_clear <= 1'b1;
                    crc_ca_match <= 1'b0;
                    crc_ca_error <= 1'b0;
                    crc_data_match <= 1'b0;
                    crc_data_error <= 1'b0;
                    command<=command_in;
                    address<=address_in;
                    wr_data<=wr_data_in;
                end
                STATE_CMD: begin
                    cs_n     <= 1'b0;
                    io_oe    <= 1'b1;
                    io_out   <= command;
                    byte_cnt <= 1;
                    // CRC for command
                    crc_ca_clear <= 1'b0;
                    crc_ca_enable <= 1'b1;
                    data_for_crc <= command;
                end
                STATE_ADDR: begin
                    io_oe    <= 1'b1;
                    case (byte_cnt)
                        1: begin io_out <= address[47:40]; data_for_crc <= address[47:40]; end
                        2: begin io_out <= address[39:32]; data_for_crc <= address[39:32]; end
                        3: begin io_out <= address[31:24]; data_for_crc <= address[31:24]; end
                        4: begin io_out <= address[23:16]; data_for_crc <= address[23:16]; end
                        5: begin io_out <= address[15:8];  data_for_crc <= address[15:8];  end
                        6: begin io_out <= address[7:0];   data_for_crc <= address[7:0];   end
                    endcase
                  if (byte_cnt<=5)
                    crc_ca_enable <= 1'b1;
                  else crc_ca_enable <= 1'b0;
                    crc_ca_clear <= 1'b0;
                    byte_cnt <= byte_cnt + 1;
                end
                STATE_SEND_CRC_CA: begin
                    io_oe <= 1'b1;
                    io_out <= crc_ca_out;
                    crc_ca_enable <= 1'b0;
                end
                STATE_RECV_CRC_CA: begin
                    io_oe <= 1'b0;
                    crc_ca_recv <= io_in;
                    if (io_in == crc_ca_out) begin
                        crc_ca_match <= 1'b1;
                        crc_ca_error <= 1'b0;
                    end else begin
                        crc_ca_match <= 1'b0;
                        crc_ca_error <= 1'b1;
                    end
                end
                STATE_WR_DATA: begin
                    io_oe    <= 1'b1;
                    case (byte_cnt)
                        7: begin io_out <= wr_data[63:56]; data_for_crc <= wr_data[63:56]; end
                        8: begin io_out <= wr_data[55:48]; data_for_crc <= wr_data[55:48]; end
                        9: begin io_out <= wr_data[47:40]; data_for_crc <= wr_data[47:40]; end
                        10: begin io_out <= wr_data[39:32]; data_for_crc <= wr_data[39:32]; end
                        11: begin io_out <= wr_data[31:24]; data_for_crc <= wr_data[31:24]; end
                        12: begin io_out <= wr_data[23:16]; data_for_crc <= wr_data[23:16]; end
                        13: begin io_out <= wr_data[15:8];  data_for_crc <= wr_data[15:8];  end
                        14: begin io_out <= wr_data[7:0];   data_for_crc <= wr_data[7:0];   end
                    endcase
                  if (byte_cnt<=13)
                    crc_data_enable <= 1'b1;
                  else crc_data_enable <= 1'b0;
                    //crc_data_enable <= 1'b1;
                    crc_data_clear <= 0;
                    byte_cnt <= byte_cnt + 1;
                end
                STATE_SEND_CRC_DATA: begin
                    crc_data_clear<=0;
                    io_oe <= 1'b1;
                    io_out <= crc_data_out;
                    crc_data_enable <= 1'b0;
                end
                STATE_RD_DATA: begin
                    crc_ca_match <= 1'b0;
                    io_oe <= 1'b0;
                   // if (data_strobe == 1'b1) begin
                        case (byte_cnt)
                            8: begin rdata_buf[63:56] <= io_in; data_for_crc <= io_in; end
                            9: begin rdata_buf[55:48] <= io_in; data_for_crc <= io_in; end
                            10: begin rdata_buf[47:40] <= io_in; data_for_crc <= io_in; end
                            11: begin rdata_buf[39:32] <= io_in; data_for_crc <= io_in; end
                            12: begin rdata_buf[31:24] <= io_in; data_for_crc <= io_in; end
                            13: begin rdata_buf[23:16] <= io_in; data_for_crc <= io_in; end
                            14: begin rdata_buf[15:8]  <= io_in; data_for_crc <= io_in; end
                            15: begin rdata_buf[7:0]   <= io_in; data_for_crc <= io_in; end
                        endcase
                  if (byte_cnt>=8 && byte_cnt<=14)
                        crc_data_enable <= 1'b1;
                  else crc_data_enable <= 1'b0;
                      crc_data_clear <= 0;
                    //end
                  if (Latency-1 > wait_cycles) begin
                        wait_cycles <= wait_cycles +1;
                    end else begin
                        byte_cnt <= byte_cnt + 1;
                    end
                end
                STATE_RECV_CRC_DATA: begin
                    io_oe <= 1'b0;
                    crc_data_recv <= io_in;
                    if (io_in == crc_data_out) begin
                        crc_data_match <= 1'b1;
                        crc_data_error <= 1'b0;
                    end else begin
                        crc_data_match <= 1'b0;
                        crc_data_error <= 1'b1;
                    end
                end
                STATE_FINISH: begin
                    if (crc_ca_error_slave == 1'b1 || crc_data_error_slave == 1'b1 || crc_data_error == 1'b1) begin
                        // Retransmission needed - reset counters and signals
                        cs_n    <= 1'b1;  // Deassert CS for retransmission
                        io_oe   <= 1'b0;
                        done    <= 1'b0;
                                    byte_cnt <= 4'd0;
                        wait_cycles <= 3'd0;
                        retransmit_cnt <= retransmit_cnt + 1;  // Increment retransmission counter
            crc_ca_clear <= 1'b1;
            crc_data_clear <= 1'b1;
            crc_ca_match <= 1'b0;
            crc_ca_error <= 1'b0;
            crc_data_match <= 1'b0;
            crc_data_error <= 1'b0;
                    end else begin
                        // Normal completion
                        crc_data_match <= 1'b0;
                        cs_n    <= 1'b1;
                        io_oe   <= 1'b0;
                        done    <= 1'b1;
                        rd_data <= rdata_buf;
                    end
                end
            endcase
        end
    end
endmodule

/**
 * @module xspi_sopi_slave
 * @brief xSPI slave for command/address/data transfer and simple memory.
 * @param clk      System clock
 * @param rst_n    Active-low reset
 * @param cs_n     Chip select (active low)
 * @param sck      SPI clock
 * @param io_out   Output to IO bus
 * @param io_in    Input from IO bus
 * @param io_oe    Output enable for IO bus
 * @param ready    Ready flag (debug/status)
 */
module xspi_sopi_slave (
    input  wire        clk,      //!< System clock
    input  wire        rst_n,    //!< Active-low reset

    input  wire        cs_n,     //!< Chip select (active low)
    input  wire        sck,      //!< SPI clock
    output reg [7:0]   io_out,   //!< Output to IO bus
    input  wire [7:0]  io_in,    //!< Input from IO bus
    output reg         io_oe,    //!< Output enable for IO bus

    output reg         ready,     //!< Ready flag (debug/status)
    output wire        data_strobe,
    output reg         crc_ca_match,
    output reg         crc_data_match,
    output reg         crc_ca_error,
    output reg         crc_data_error,
    input wire         crc_ca_error_master,
    input wire         crc_data_error_master
);

    // === FSM States ===
    reg [3:0] state;         //!< Current FSM state
    localparam STATE_IDLE         = 4'd0; //!< Idle state
    localparam STATE_CMD          = 4'd1; //!< Receive command
    localparam STATE_ADDR         = 4'd2; //!< Receive address
    localparam STATE_RECV_CRC_CA  = 4'd3; //!< Receive CRC for command/address
    localparam STATE_WAIT_LATENCY = 4'd4; //!< Wait latency for read
    localparam STATE_WR_DATA      = 4'd5; //!< Receive write data
    localparam STATE_RECV_CRC_DATA= 4'd6; //!< Receive CRC for data
    localparam STATE_RD_DATA      = 4'd7; //!< Send read data
    localparam STATE_SEND_CRC_DATA= 4'd8; //!< Send CRC for data
    localparam STATE_DONE         = 4'd9; //!< Done state
    reg [3:0] byte_cnt;
    reg [2:0] latency_cnt;
    reg [3:0] retransmit_cnt;  // Retransmission counter for slave
    reg [7:0]  command_reg;
    reg [47:0] addr_reg;
    reg [63:0] data_reg;
    reg [63:0] mem;
    reg crc_ca_clear, crc_ca_enable;
    wire [7:0] crc_ca_out;
    reg [7:0] crc_ca_recv;
    reg crc_data_clear, crc_data_enable;
    wire [7:0] crc_data_out;
    reg [7:0] crc_data_recv;
    reg [7:0] data_for_crc;
  assign data_strobe = (state == STATE_RD_DATA || STATE_SEND_CRC_DATA) ? clk : 1'b0;
    crc8 crc_ca_inst (
        .clk(clk),
        .rst(!rst_n),
        .enable(crc_ca_enable),
        .clear(crc_ca_clear),
        .data_in(data_for_crc),
        .crc_out(crc_ca_out)
    );
    crc8_slave crc_data_inst (
        .clk(clk),
        .rst(!rst_n),
        .enable(crc_data_enable),
        .clear(crc_data_clear),
        .data_in(data_for_crc),
        .crc_out(crc_data_out)
    );
    always @(negedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= STATE_IDLE;
            io_out      <= 8'h00;
            io_oe       <= 1'b0;
            byte_cnt    <= 4'd0;
            latency_cnt <= 3'd0;
            retransmit_cnt <= 4'd0;
            ready       <= 1'b0;
            command_reg <= 8'h00;
            addr_reg    <= 48'h0;
            data_reg    <= 64'h0;
            crc_ca_clear <= 1'b1;
            crc_ca_enable <= 1'b0;
            crc_data_clear <= 1'b1;
            crc_data_enable <= 1'b0;
            crc_ca_match <= 1'b0;
            crc_ca_error <= 1'b0;
            crc_data_match <= 1'b0;
            crc_data_error <= 1'b0;
        end else begin
            crc_ca_enable <= 1'b0;
            crc_data_enable <= 1'b0;
            case (state)
                STATE_IDLE: begin
                    ready <= 0;
                    byte_cnt <= 0;
                    io_oe <= 1'b0;
                    crc_ca_clear <= 1'b1;
                    crc_data_clear <= 1'b1;
                    crc_ca_match <= 1'b0;
                    crc_ca_error <= 1'b0;
                    crc_data_match <= 1'b0;
                    crc_data_error <= 1'b0;
                    // Only transition to CMD if CS_n is asserted
                    //if (!cs_n) begin
                        state <= STATE_CMD;
                   // end
                end
                STATE_CMD: begin
                    if (!cs_n) begin
                        command_reg <= io_in;
                        byte_cnt    <= 1;
                        state       <= STATE_ADDR;
                        crc_ca_clear <= 1'b0;
                        crc_ca_enable <= 1'b1;
                        data_for_crc <= io_in;
                    end //else begin
                       // state <= STATE_IDLE; // Return to idle if CS_n is high
                   // end
                end
                STATE_ADDR: begin
                    case (byte_cnt)
                        1: begin addr_reg[47:40] <= io_in; data_for_crc <= io_in; end
                        2: begin addr_reg[39:32] <= io_in; data_for_crc <= io_in; end
                        3: begin addr_reg[31:24] <= io_in; data_for_crc <= io_in; end
                        4: begin addr_reg[23:16] <= io_in; data_for_crc <= io_in; end
                        5: begin addr_reg[15:8]  <= io_in; data_for_crc <= io_in; end
                        6: begin addr_reg[7:0]   <= io_in; data_for_crc <= io_in; end
                    endcase
                  if (byte_cnt<=5)
                    crc_ca_enable <= 1'b1;
                  else crc_ca_enable <= 1'b0;
                    //crc_ca_enable <= 1'b1;
                    crc_ca_clear <= 1'b0;
                    byte_cnt <= byte_cnt + 1;
                    if (byte_cnt == 6) state <= STATE_RECV_CRC_CA;
                end
                STATE_RECV_CRC_CA: begin
                    crc_ca_recv <= io_in;
                    if (io_in == crc_ca_out) begin
                      if (retransmit_cnt < 3) begin
                        crc_ca_match <= 1'b0;
                        crc_ca_error <= 1'b1;
                      end
                      else begin
                        crc_ca_match <= 1'b1;
                        crc_ca_error <= 1'b0;
                      end
                    end else begin
                        crc_ca_match <= 1'b0;
                        crc_ca_error <= 1'b1;
                    end
                    if (command_reg == 8'hFF) begin
                        latency_cnt <= 0;
                        state <= STATE_WAIT_LATENCY;
                    end else if (command_reg == 8'hA5) begin
                        state <= STATE_WR_DATA;
                    end else begin
                        state <= STATE_DONE;
                    end
                end
                STATE_WAIT_LATENCY: begin
                  crc_ca_match<=0;
                    latency_cnt <= latency_cnt + 1;
                    if (latency_cnt == 3'd5) begin
                        data_reg <= mem;
                        byte_cnt <= 7;
                        io_oe    <= 1'b1;
                        state    <= STATE_RD_DATA;
                        crc_data_clear <= 1'b1;
                    end
                end
                STATE_WR_DATA: begin
                  crc_ca_match<=0;
                    case (byte_cnt)
                        7:  begin data_reg[63:56] <= io_in; data_for_crc <= io_in; end
                        8:  begin data_reg[55:48] <= io_in; data_for_crc <= io_in; end
                        9:  begin data_reg[47:40] <= io_in; data_for_crc <= io_in; end
                        10: begin data_reg[39:32] <= io_in; data_for_crc <= io_in; end
                        11: begin data_reg[31:24] <= io_in; data_for_crc <= io_in; end
                        12: begin data_reg[23:16] <= io_in; data_for_crc <= io_in; end
                        13: begin data_reg[15:8]  <= io_in; data_for_crc <= io_in; end
                        14: begin data_reg[7:0]   <= io_in; data_for_crc <= io_in; end
                    endcase
                   if (byte_cnt>=7 && byte_cnt<=13)
                    crc_data_enable <= 1'b1;
                  else crc_data_enable <= 1'b0;
                    crc_data_clear <= 0;
                    byte_cnt <= byte_cnt + 1;
                    if (byte_cnt == 14) state <= STATE_RECV_CRC_DATA;
                end
                STATE_RECV_CRC_DATA: begin
                    crc_data_clear<=0;
                    crc_data_recv <= io_in;
                    if (io_in == crc_data_out) begin
                        crc_data_match <= 1'b1;
                        crc_data_error <= 1'b0;
                    end else begin
                        crc_data_match <= 1'b0;
                        crc_data_error <= 1'b1;
                    end
                    mem <= data_reg;
                    state <= STATE_DONE;
                end
                STATE_RD_DATA: begin
                    case (byte_cnt)
                        7:  begin io_out <= data_reg[63:56]; data_for_crc <= data_reg[63:56]; end
                        8:  begin io_out <= data_reg[55:48]; data_for_crc <= data_reg[55:48]; end
                        9:  begin io_out <= data_reg[47:40]; data_for_crc <= data_reg[47:40]; end
                        10: begin io_out <= data_reg[39:32]; data_for_crc <= data_reg[39:32]; end
                        11: begin io_out <= data_reg[31:24]; data_for_crc <= data_reg[31:24]; end
                        12: begin io_out <= data_reg[23:16]; data_for_crc <= data_reg[23:16]; end
                        13: begin io_out <= data_reg[15:8];  data_for_crc <= data_reg[15:8];  end
                        14: begin io_out <= data_reg[7:0];   data_for_crc <= data_reg[7:0];   end
                    endcase
                  if (byte_cnt>=7 && byte_cnt<=13)
                    crc_data_enable <= 1'b1;
                  else crc_data_enable <= 1'b0;
                    crc_data_clear <= 0;
                    byte_cnt <= byte_cnt + 1;
                  if (byte_cnt == 14) state <= STATE_SEND_CRC_DATA;
                end
                STATE_SEND_CRC_DATA: begin
                    io_out <= crc_data_out;
                    io_oe <= 1'b1;
                    crc_data_enable <= 1'b0;
                    state <= STATE_DONE;
                end
                STATE_DONE: begin
                    crc_data_match <= 1'b0;
                    io_oe <= 1'b0;
                    
                    // Check for retransmission conditions
                    if ((crc_ca_error_master == 1'b1 || crc_data_error_master == 1'b1 || crc_ca_error == 1'b1 || crc_data_error == 1'b1) && retransmit_cnt < 4'd3) begin
                        // Reset for retransmission
                        byte_cnt <= 4'd0;
                        latency_cnt <= 3'd0;
                        retransmit_cnt <= retransmit_cnt + 1;
                        crc_ca_clear <= 1'b1;
                        crc_data_clear <= 1'b1;
                        crc_ca_match <= 1'b0;
                        crc_ca_error <= 1'b0;
                        crc_data_match <= 1'b0;
                        crc_data_error <= 1'b0;
                        state <= STATE_CMD;
                        ready <= 1'b0;
                    end else begin
                        state <= STATE_IDLE;
                        ready <= 1'b1;
                    end
                end
                default: state <= STATE_IDLE;
            endcase
        end
    end
endmodule

// CRC8 module (as provided)
module crc8 (
    input  wire       clk,      ///< System clock
    input  wire       rst,      ///< Asynchronous reset
    input  wire       enable,   ///< Enable CRC calculation
    input  wire       clear,    ///< Clear CRC output
    input  wire [7:0] data_in,  ///< 8-bit input data
    output reg  [7:0] crc_out   ///< 8-bit CRC output
);
    parameter POLY = 8'h07; ///< CRC polynomial
    wire [7:0] crc_in = crc_out ^ data_in;
    wire [7:0] stage0 = (crc_in[7]) ? (crc_in << 1) ^ POLY : (crc_in << 1);
    wire [7:0] stage1 = (stage0[7]) ? (stage0 << 1) ^ POLY : (stage0 << 1);
    wire [7:0] stage2 = (stage1[7]) ? (stage1 << 1) ^ POLY : (stage1 << 1);
    wire [7:0] stage3 = (stage2[7]) ? (stage2 << 1) ^ POLY : (stage2 << 1);
    wire [7:0] stage4 = (stage3[7]) ? (stage3 << 1) ^ POLY : (stage3 << 1);
    wire [7:0] stage5 = (stage4[7]) ? (stage4 << 1) ^ POLY : (stage4 << 1);
    wire [7:0] stage6 = (stage5[7]) ? (stage5 << 1) ^ POLY : (stage5 << 1);
    wire [7:0] crc_next = (stage6[7]) ? (stage6 << 1) ^ POLY : (stage6 << 1);
    always @(posedge clk or posedge rst) begin
        if (rst)
            crc_out <= 8'h00;
        else if (clear)
            crc_out <= 8'h00;
        else if (enable)
            crc_out <= crc_next;
    end
endmodule 

// CRC8 module (as provided)
module crc8_slave (
    input  wire       clk,      ///< System clock
    input  wire       rst,      ///< Asynchronous reset
    input  wire       enable,   ///< Enable CRC calculation
    input  wire       clear,    ///< Clear CRC output
    input  wire [7:0] data_in,  ///< 8-bit input data
    output reg  [7:0] crc_out   ///< 8-bit CRC output
);
    parameter POLY = 8'h07; ///< CRC polynomial
    wire [7:0] crc_in = crc_out ^ data_in;
    wire [7:0] stage0 = (crc_in[7]) ? (crc_in << 1) ^ POLY : (crc_in << 1);
    wire [7:0] stage1 = (stage0[7]) ? (stage0 << 1) ^ POLY : (stage0 << 1);
    wire [7:0] stage2 = (stage1[7]) ? (stage1 << 1) ^ POLY : (stage1 << 1);
    wire [7:0] stage3 = (stage2[7]) ? (stage2 << 1) ^ POLY : (stage2 << 1);
    wire [7:0] stage4 = (stage3[7]) ? (stage3 << 1) ^ POLY : (stage3 << 1);
    wire [7:0] stage5 = (stage4[7]) ? (stage4 << 1) ^ POLY : (stage4 << 1);
    wire [7:0] stage6 = (stage5[7]) ? (stage5 << 1) ^ POLY : (stage5 << 1);
    wire [7:0] crc_next = (stage6[7]) ? (stage6 << 1) ^ POLY : (stage6 << 1);
  always @(negedge clk or posedge rst) begin
        if (rst)
            crc_out <= 8'h00;
        else if (clear)
            crc_out <= 8'h00;
        else if (enable)
            crc_out <= crc_next;
    end
endmodule 



