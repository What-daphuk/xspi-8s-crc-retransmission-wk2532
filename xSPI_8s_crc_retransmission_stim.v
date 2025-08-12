module xspi_stimulus (
    output  reg clk,
    output reg  rst_n,
    output reg  start,
    output reg [7:0]  command,
    output reg [47:0] address,
    output reg [63:0] wr_data,
    input wire [63:0] rd_data,
    input  wire done,
    input wire ready,
    input wire crc_ca_match_slave,
    input wire crc_ca_error_slave,
    input wire crc_data_match_master,
    input wire crc_data_error_master,
    input wire crc_data_match_slave,
    input wire crc_data_error_slave,
    input wire crc_ca_match_master,
    input wire crc_ca_error_master
);

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end


    initial begin
         // $dumpfile("xspi_tb.vcd");
   // $dumpvars(0, tb_xspi);
        $display("=== xSPI Master/Slave Simulation Start ===");
        rst_n = 0;
        #20;
        rst_n = 1;
        #20;

        // --- WRITE ---
        @(posedge clk);
        command = 8'hA5;
        address = 48'h6655443322AB;
        wr_data = 64'h1122334455667788;
        //rw = 0;
        start = 1;

        @(posedge clk);
        start = 0;

        wait (done);
        $display("[WRITE] Wrote %h to address %h", wr_data, address);
        $display("[WRITE] CRC CA match_slave: %b, CRC CA error_slave: %b, CRC DATA match_master: %b, CRC DATA error_master: %b, CRC DATA match_slave: %b, CRC DATA error_slave: %b", crc_ca_match_slave, crc_ca_error_slave, crc_data_match_master, crc_data_error_master, crc_data_match_slave, crc_data_error_slave);

        #50;

        // --- READ ---
        @(posedge clk);
        command = 8'hFF;
        address = 48'h6655443322AB;
        wr_data = 64'h0;
       // rw = 1;
        @(posedge clk);
        start = 1;
        @(posedge clk);
      @(posedge clk);
        start = 0;

        wait (done);
        $display("[READ] Read %h from address %h", rd_data, address);
        $display("[WRITE] CRC CA match_slave: %b, CRC CA error_slave: %b, CRC DATA match_master: %b, CRC DATA error_master: %b, CRC DATA match_slave: %b, CRC DATA error_slave: %b", crc_ca_match_slave, crc_ca_error_slave, crc_data_match_master, crc_data_error_master, crc_data_match_slave, crc_data_error_slave);

        if (rd_data == 64'h1122334455667788) begin
            $display("✅ PASS: Read data matches written data.");
        end else begin
            $display("❌ FAIL: Read data mismatch!");
        end

        #100;
        $finish;
    end
  initial begin 
    #2000;
    $finish;
  end

endmodule
