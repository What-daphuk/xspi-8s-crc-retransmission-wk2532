`timescale 1ns/1ps

module xspi_monitor (
    // Inputs from DUT and Stimulus
    input wire clk,
    input wire rst_n,
    input wire done,
    input wire [7:0]  command,   // The command for the current/last transaction
    input wire [47:0] address,   // The address for the current/last transaction
    input wire [63:0] wr_data,   // The data driven by the stimulus
    input [63:0] rd_data,   // The data returned by the DUT
    // CRC signals
    input wire crc_ca_match_slave,
    input wire crc_ca_error_slave,
    input wire crc_data_match_master,
    input wire crc_data_error_master,
    input wire crc_data_match_slave,
    input wire crc_data_error_slave,
    input wire crc_ca_match_master,
    input wire crc_ca_error_master
);

    // Internal storage to remember the last write operation
    // This allows us to verify the subsequent read operation.
    reg [63:0] last_wr_data;
    reg [47:0] last_address;

    // The main checking logic, triggered when a transaction completes.
    // We use @(posedge done) because 'done' is a single-cycle pulse.
    always @(posedge done) begin
        if (rst_n) begin
            // Check which operation was performed based on the command code
            // provided by the stimulus.

            // --- WRITE Operation Check ---
            if (command == 8'hA5) begin
                $display("--------------------------------------------------");
                $display("[MONITOR] WRITE Transaction Report");
                $display("  Address:    %h", address);
                $display("  Wrote Data: %h", wr_data);
                $display("  Slave CA CRC Status:   Match=%b, Error=%b", crc_ca_match_slave, crc_ca_error_slave);
                $display("  Slave Data CRC Status: Match=%b, Error=%b", crc_data_match_slave, crc_data_error_slave);
                $display("--------------------------------------------------");

                // Store the written data and address for later read verification
                last_wr_data <= wr_data;
                last_address <= address;
            end

            // --- READ Operation Check ---
            else if (command == 8'hFF) begin
                $display("--------------------------------------------------");
                $display("[MONITOR] READ Transaction Report");
                $display("  Address:       %h", address);
                $display("  Read Data:     %h", rd_data);
                $display("  Expected Data: %h", last_wr_data);
                $display("  Slave CA CRC Status:    Match=%b, Error=%b", crc_ca_match_slave, crc_ca_error_slave);
                $display("  Master Data CRC Status: Match=%b, Error=%b", crc_data_match_master, crc_data_error_master);

                // Verify that the read address matches the last write address
                if (address == last_address) begin
                    // Verify that the read data matches the last written data
                    if (rd_data == last_wr_data) begin
                        $display("✅ [MONITOR] PASS: Read data matches written data.");
                    end else begin
                        $display("❌ [MONITOR] FAIL: Read data mismatch!");
                    end
                end else begin
                    $display("❌ [MONITOR] FAIL: Read address mismatch!");
                end
                $display("--------------------------------------------------");
            end
        end
    end

endmodule
