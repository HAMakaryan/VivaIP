
`timescale 1 ns / 1 ps

  module Reg_Ltc_v1_0_S00_AXI #
  (
    // Users to add parameters here

    // User parameters ends
    // Do not modify the parameters beyond this line
    parameter   integer CHANNEL_NUM       = 6,
    // Width of S_AXI data bus
    parameter integer C_S_AXI_DATA_WIDTH  = 32,
    // Width of S_AXI address bus
    parameter integer C_S_AXI_ADDR_WIDTH  = 6
  )
  (
    // Users to add ports here
//    input   wire        sclk_i,
    input   wire        sdo_i,
    inout   wire        conv_io,
    output  wire        conv_o,
    output  wire        sck_o,
    output  wire        gen_o,

    output  wire        trigger_input_1,
    output  wire        isb,
    output  wire        g_tia,
    output  wire        msb,
    output  wire        latch,
    output  wire        lsb,
    output  wire        RS485_Rx_En_n,
    output  wire        RS485_Tx_En,

    // User ports ends
    // Do not modify the ports beyond this line

    // Global Clock Signal
    input wire  S_AXI_ACLK,
    // Global Reset Signal. This Signal is Active LOW
    input wire  S_AXI_ARESETN,
    // Write address (issued by master, acceped by Slave)
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,
    // Write channel Protection type. This signal indicates the
        // privilege and security level of the transaction, and whether
        // the transaction is a data access or an instruction access.
    input wire [2 : 0] S_AXI_AWPROT,
    // Write address valid. This signal indicates that the master signaling
        // valid write address and control information.
    input wire  S_AXI_AWVALID,
    // Write address ready. This signal indicates that the slave is ready
        // to accept an address and associated control signals.
    output wire  S_AXI_AWREADY,
    // Write data (issued by master, acceped by Slave)
    input wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,
    // Write strobes. This signal indicates which byte lanes hold
        // valid data. There is one write strobe bit for each eight
        // bits of the write data bus.
    input wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
    // Write valid. This signal indicates that valid write
        // data and strobes are available.
    input wire  S_AXI_WVALID,
    // Write ready. This signal indicates that the slave
        // can accept the write data.
    output wire  S_AXI_WREADY,
    // Write response. This signal indicates the status
        // of the write transaction.
    output wire [1 : 0] S_AXI_BRESP,
    // Write response valid. This signal indicates that the channel
        // is signaling a valid write response.
    output wire  S_AXI_BVALID,
    // Response ready. This signal indicates that the master
        // can accept a write response.
    input wire  S_AXI_BREADY,
    // Read address (issued by master, acceped by Slave)
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,
    // Protection type. This signal indicates the privilege
        // and security level of the transaction, and whether the
        // transaction is a data access or an instruction access.
    input wire [2 : 0] S_AXI_ARPROT,
    // Read address valid. This signal indicates that the channel
        // is signaling valid read address and control information.
    input wire  S_AXI_ARVALID,
    // Read address ready. This signal indicates that the slave is
        // ready to accept an address and associated control signals.
    output wire  S_AXI_ARREADY,
    // Read data (issued by slave)
    output wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,
    // Read response. This signal indicates the status of the
        // read transfer.
    output wire [1 : 0] S_AXI_RRESP,
    // Read valid. This signal indicates that the channel is
        // signaling the required read data.
    output wire  S_AXI_RVALID,
    // Read ready. This signal indicates that the master can
        // accept the read data and response information.
    input wire  S_AXI_RREADY
  );

  // AXI4LITE signals
  reg [C_S_AXI_ADDR_WIDTH-1 : 0]  axi_awaddr;
  reg   axi_awready;
  reg   axi_wready;
  reg [1 : 0]   axi_bresp;
  reg   axi_bvalid;
  reg [C_S_AXI_ADDR_WIDTH-1 : 0]  axi_araddr;
  reg   axi_arready;
  reg [C_S_AXI_DATA_WIDTH-1 : 0]  axi_rdata;
  reg [1 : 0]   axi_rresp;
  reg   axi_rvalid;

  // Example-specific design signals
  // local parameter for addressing 32 bit / 64 bit C_S_AXI_DATA_WIDTH
  // ADDR_LSB is used for addressing 32/64 bit registers/memories
  // ADDR_LSB = 2 for 32 bits (n downto 2)
  // ADDR_LSB = 3 for 64 bits (n downto 3)
  localparam integer ADDR_LSB = (C_S_AXI_DATA_WIDTH/32) + 1;
  localparam integer OPT_MEM_ADDR_BITS = 3;
  //----------------------------------------------
  localparam  integer BIT_NUM           = CHANNEL_NUM * 16 + 2;
  localparam          IDLE_STATE        = 2'b00;
  localparam          READ_DATA_STATE   = 2'b01;
  localparam          GEN_CONV_STATE    = 2'b10;

  wire                sck_posedge;
  wire                sclk;
  wire                tris_O;
//  wire                tris_IO;
//  wire                tris_I;
  wire                tris_T;
  wire                start_conv;

  reg   [15:0]        input_data_reg;
  reg                 sclk_reg;
  reg   [ 1:0]        state_reg;
  reg   [ 1:0]        state_next;
  reg                 conv_internal;
  reg   [ 6:0]        bit_counter;
  reg                 sck_enable;
  reg   [ 1:0]        sck_counter;
  reg   [19:0]        counter;
  reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg1_reg;
  reg                 tris_O_reg;
  reg                 sdo_reg;

  //-- Signals for user logic register space example
  //------------------------------------------------
  //-- Number of Slave Registers 16
  reg [C_S_AXI_DATA_WIDTH-1:0]  status_reg;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg1;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg2;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg3;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg4;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg5;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg6;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg7;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg8;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg9;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg10;
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg11;      // (inpt_data_reg)
  reg [C_S_AXI_DATA_WIDTH-1:0]  slv_reg12;      // (output_data_reg)
  reg [C_S_AXI_DATA_WIDTH-1:0]  ch01_data_reg;
  reg [C_S_AXI_DATA_WIDTH-1:0]  ch23_data_reg;
  reg [C_S_AXI_DATA_WIDTH-1:0]  ch45_data_reg;
  wire   slv_reg_rden;
  wire   slv_reg_wren;
  reg [C_S_AXI_DATA_WIDTH-1:0]   reg_data_out;
  integer  byte_index;
  reg  aw_en;

  ////////////////USER SIGNALS BEGIN
/*

status_reg
  status_reg[0]
          1 = there is a new data in ch01_data_reg[15: 0] at ch0 input
          0 = the last data has been read from register ch01_data_reg

  status_reg[1]
          1 = there is a new data in ch01_data_reg[31:16] at ch1 input
          0 = the last data has been read from register ch01_data_reg

  status_reg[2]
          1 = there is a new data in ch23_data_reg[15: 0] at ch2 input
          0 = the last data has been read from register ch23_data_reg

  status_reg[3]
          1 = there is a new data in ch23_data_reg[31:16] at ch3 input
          0 = the last data has been read from register ch23_data_reg

  status_reg[4]
          1 = there is a new data in ch45_data_reg[15: 0] at ch4 input
          0 = the last data has been read from register ch45_data_reg

  status_reg[5]
          1 = there is a new data in ch45_data_reg[31:16] at ch5 input
          0 = the last data has been read from register ch45_data_reg


slv_reg1
  changing the value of this register generates a new measurement cycle

slv_reg12 (output_data_reg)

    isb             = slv_reg12[0];
    g_tia           = slv_reg12[1];
    msb             = slv_reg12[2];
    latch           = slv_reg12[3];
    lsb             = slv_reg12[4];
    trigger_input_1 = slv_reg12[5];
    RS485_Rx_En_n   = slv_reg12[6];
    RS485_Tx_En     =!slv_reg12[7];


*/

  assign  isb             = slv_reg12[0];
  assign  g_tia           = slv_reg12[1];
  assign  msb             = slv_reg12[2];
  assign  latch           = slv_reg12[3];
  assign  lsb             = slv_reg12[4];
  assign  trigger_input_1 = slv_reg12[5];
  assign  RS485_Rx_En_n   = slv_reg12[6];
  assign  RS485_Tx_En     =!slv_reg12[7];


  ////////////////USER SIGNALS END
  // I/O Connections assignments

  assign S_AXI_AWREADY  = axi_awready;
  assign S_AXI_WREADY = axi_wready;
  assign S_AXI_BRESP  = axi_bresp;
  assign S_AXI_BVALID = axi_bvalid;
  assign S_AXI_ARREADY  = axi_arready;
  assign S_AXI_RDATA  = axi_rdata;
  assign S_AXI_RRESP  = axi_rresp;
  assign S_AXI_RVALID = axi_rvalid;
  // Implement axi_awready generation
  // axi_awready is asserted for one S_AXI_ACLK clock cycle when both
  // S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_awready is
  // de-asserted when reset is low.

  always @( posedge S_AXI_ACLK )
  begin
    if ( S_AXI_ARESETN == 1'b0 )
      begin
        axi_awready <= 1'b0;
        aw_en <= 1'b1;
      end
    else
      begin
        if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en)
          begin
            // slave is ready to accept write address when
            // there is a valid write address and write data
            // on the write address and data bus. This design
            // expects no outstanding transactions.
            axi_awready <= 1'b1;
            aw_en <= 1'b0;
          end
          else if (S_AXI_BREADY && axi_bvalid)
              begin
                aw_en <= 1'b1;
                axi_awready <= 1'b0;
              end
        else
          begin
            axi_awready <= 1'b0;
          end
      end
  end

  // Implement axi_awaddr latching
  // This process is used to latch the address when both
  // S_AXI_AWVALID and S_AXI_WVALID are valid.

  always @( posedge S_AXI_ACLK )
  begin
    if ( S_AXI_ARESETN == 1'b0 )
      begin
        axi_awaddr <= 0;
      end
    else
      begin
        if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en)
          begin
            // Write Address latching
            axi_awaddr <= S_AXI_AWADDR;
          end
      end
  end

  // Implement axi_wready generation
  // axi_wready is asserted for one S_AXI_ACLK clock cycle when both
  // S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_wready is
  // de-asserted when reset is low.

  always @( posedge S_AXI_ACLK )
  begin
    if ( S_AXI_ARESETN == 1'b0 )
      begin
        axi_wready <= 1'b0;
      end
    else
      begin
        if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en )
          begin
            // slave is ready to accept write data when
            // there is a valid write address and write data
            // on the write address and data bus. This design
            // expects no outstanding transactions.
            axi_wready <= 1'b1;
          end
        else
          begin
            axi_wready <= 1'b0;
          end
      end
  end

  // Implement memory mapped register select and write logic generation
  // The write data is accepted and written to memory mapped registers when
  // axi_awready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted. Write strobes are used to
  // select byte enables of slave registers while writing.
  // These registers are cleared when reset (active low) is applied.
  // Slave register write enable is asserted when valid address and data are available
  // and the slave is ready to accept the write address and write data.
  assign slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

  always @( posedge S_AXI_ACLK )
  begin
    if ( S_AXI_ARESETN == 1'b0 )
      begin
//        slv_reg0 <= 0;
        slv_reg1 <= 0;
        slv_reg2 <= 0;
        slv_reg3 <= 0;
        slv_reg4 <= 0;
        slv_reg5 <= 0;
        slv_reg6 <= 0;
        slv_reg7 <= 0;
        slv_reg8 <= 0;
        slv_reg9 <= 0;
        slv_reg10 <= 0;
//        slv_reg11 [C_S_AXI_DATA_WIDTH-1:1] <= 0;
        slv_reg12 <= 0;
//        ch01_data_reg <= 0;
//        ch23_data_reg <= 0;
//        ch45_data_reg <= 0;
      end
    else begin
      if (slv_reg_wren)
        begin
          case ( axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] )
            4'h0:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 0
//                  slv_reg0[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'h1:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 1
                  slv_reg1[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'h2:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 2
                  slv_reg2[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'h3:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 3
                  slv_reg3[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'h4:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 4
                  slv_reg4[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'h5:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 5
                  slv_reg5[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'h6:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 6
                  slv_reg6[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'h7:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 7
                  slv_reg7[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'h8:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 8
                  slv_reg8[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'h9:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 9
                  slv_reg9[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'hA:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 10
                  slv_reg10[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'hB:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 11
                  if (byte_index == 0) begin
//                    slv_reg11[(byte_index*8) +: 7] <= S_AXI_WDATA[(byte_index*8) +: 7];
                  end else begin
//                    slv_reg11[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                  end
                end
            4'hC:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 12
                  slv_reg12[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'hD:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 13
//                  ch01_data_reg[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'hE:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 14
//                  ch23_data_reg[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            4'hF:
              for ( byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1 )
                if ( S_AXI_WSTRB[byte_index] == 1 ) begin
                  // Respective byte enables are asserted as per write strobes
                  // Slave register 15
//                  ch45_data_reg[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                end
            default : begin
//                        slv_reg0 <= slv_reg0;
                        slv_reg1 <= slv_reg1;
                        slv_reg2 <= slv_reg2;
                        slv_reg3 <= slv_reg3;
                        slv_reg4 <= slv_reg4;
                        slv_reg5 <= slv_reg5;
                        slv_reg6 <= slv_reg6;
                        slv_reg7 <= slv_reg7;
                        slv_reg8 <= slv_reg8;
                        slv_reg9 <= slv_reg9;
                        slv_reg10 <= slv_reg10;
//                        slv_reg11[C_S_AXI_DATA_WIDTH-1:1] <= slv_reg11[C_S_AXI_DATA_WIDTH-1:1];
                        slv_reg12 <= slv_reg12;
//                        ch01_data_reg <= ch01_data_reg;
//                        ch23_data_reg <= ch23_data_reg;
//                        ch45_data_reg <= ch45_data_reg;
                      end
          endcase
        end
    end
  end

  // Implement write response logic generation
  // The write response and response valid signals are asserted by the slave
  // when axi_wready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted.
  // This marks the acceptance of address and indicates the status of
  // write transaction.

  always @( posedge S_AXI_ACLK )
  begin
    if ( S_AXI_ARESETN == 1'b0 )
      begin
        axi_bvalid  <= 0;
        axi_bresp   <= 2'b0;
      end
    else
      begin
        if (axi_awready && S_AXI_AWVALID && ~axi_bvalid && axi_wready && S_AXI_WVALID)
          begin
            // indicates a valid write response is available
            axi_bvalid <= 1'b1;
            axi_bresp  <= 2'b0; // 'OKAY' response
          end                   // work error responses in future
        else
          begin
            if (S_AXI_BREADY && axi_bvalid)
              //check if bready is asserted while bvalid is high)
              //(there is a possibility that bready is always asserted high)
              begin
                axi_bvalid <= 1'b0;
              end
          end
      end
  end

  // Implement axi_arready generation
  // axi_arready is asserted for one S_AXI_ACLK clock cycle when
  // S_AXI_ARVALID is asserted. axi_awready is
  // de-asserted when reset (active low) is asserted.
  // The read address is also latched when S_AXI_ARVALID is
  // asserted. axi_araddr is reset to zero on reset assertion.

  always @( posedge S_AXI_ACLK )
  begin
    if ( S_AXI_ARESETN == 1'b0 )
      begin
        axi_arready <= 1'b0;
        axi_araddr  <= 32'b0;
      end
    else
      begin
        if (~axi_arready && S_AXI_ARVALID)
          begin
            // indicates that the slave has acceped the valid read address
            axi_arready <= 1'b1;
            // Read address latching
            axi_araddr  <= S_AXI_ARADDR;
          end
        else
          begin
            axi_arready <= 1'b0;
          end
      end
  end

  // Implement axi_arvalid generation
  // axi_rvalid is asserted for one S_AXI_ACLK clock cycle when both
  // S_AXI_ARVALID and axi_arready are asserted. The slave registers
  // data are available on the axi_rdata bus at this instance. The
  // assertion of axi_rvalid marks the validity of read data on the
  // bus and axi_rresp indicates the status of read transaction.axi_rvalid
  // is deasserted on reset (active low). axi_rresp and axi_rdata are
  // cleared to zero on reset (active low).
  always @( posedge S_AXI_ACLK )
  begin
    if ( S_AXI_ARESETN == 1'b0 )
      begin
        axi_rvalid <= 0;
        axi_rresp  <= 0;
      end
    else
      begin
        if (axi_arready && S_AXI_ARVALID && ~axi_rvalid)
          begin
            // Valid read data is available at the read data bus
            axi_rvalid <= 1'b1;
            axi_rresp  <= 2'b0; // 'OKAY' response
          end
        else if (axi_rvalid && S_AXI_RREADY)
          begin
            // Read data is accepted by the master
            axi_rvalid <= 1'b0;
          end
      end
  end

  // Implement memory mapped register select and read logic generation
  // Slave register read enable is asserted when valid address is available
  // and the slave is ready to accept the read address.
  assign slv_reg_rden = axi_arready & S_AXI_ARVALID & ~axi_rvalid;
  always @(*)
  begin
        // Address decoding for reading registers
        case ( axi_araddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] )
          4'h0   : reg_data_out <= status_reg;
          4'h1   : reg_data_out <= slv_reg1;
          4'h2   : reg_data_out <= slv_reg2;
          4'h3   : reg_data_out <= slv_reg3;
          4'h4   : reg_data_out <= slv_reg4;
          4'h5   : reg_data_out <= slv_reg5;
          4'h6   : reg_data_out <= slv_reg6;
          4'h7   : reg_data_out <= slv_reg7;
          4'h8   : reg_data_out <= slv_reg8;
          4'h9   : reg_data_out <= slv_reg9;
          4'hA   : reg_data_out <= slv_reg10;
          4'hB   : reg_data_out <= slv_reg11;
          4'hC   : reg_data_out <= slv_reg12;
          4'hD   : reg_data_out <= ch01_data_reg;
          4'hE   : reg_data_out <= ch23_data_reg;
          4'hF   : reg_data_out <= ch45_data_reg;
          default: reg_data_out <= 0;
        endcase
  end

  // Output register or memory read data
  always @( posedge S_AXI_ACLK )
  begin
    if ( S_AXI_ARESETN == 1'b0 )
      begin
        axi_rdata  <= 0;
      end
    else
      begin
        // When there is a valid read address (S_AXI_ARVALID) with
        // acceptance of read address by the slave (axi_arready),
        // output the read dada
        if (slv_reg_rden)
          begin
            axi_rdata <= reg_data_out;     // register read data
          end
      end
  end

// Add user logic here






// IOBUF: Single-ended Bi-directional Buffer
// All devices
// Xilinx HDL Language Template, version 2019.1
IOBUF

// #(
//  .DRIVE        (12         ), // Specify the output drive strength
//  .IBUF_LOW_PWR ("TRUE"     ), // Low Power - "TRUE", High Performance = "FALSE"
//  .IOSTANDARD   ("LVCMOS33" ), // Specify the I/O standard
//  .SLEW         ("FAST"     ) // Specify the output slew rate
//)

IOBUF_inst (
  .O(tris_O),   // Buffer output
  .IO(conv_io), // Buffer inout port (connect directly to top-level port)
  .I(1'b1),     // Buffer input
  .T(tris_T)    // 3-state enable input, high=input, low=output
);
// End of IOBUF_inst instantiation

assign tris_T = (state_reg == GEN_CONV_STATE)? 1'b0 : 1'b1;
assign conv_o = (counter[19:5] == 15'H7FFF)? 1'b1 : 1'b0;
assign sclk   = sck_counter[1];
assign sck_o  = sclk_reg;
assign gen_o  = counter[0];
assign start_conv = (slv_reg1_reg == slv_reg1)? 1'b0 : 1'b1;
assign sck_posedge  = ((sclk_reg == 1'b0) && (sclk == 1'b1))? 1'b1 : 1'b0;

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    ch01_data_reg   <= 32'd0;
    ch23_data_reg   <= 32'd0;
    ch45_data_reg   <= 32'd0;
    status_reg      <= {C_S_AXI_DATA_WIDTH{1'b0}};
  end else begin
    if (axi_rvalid && S_AXI_RREADY) begin
      case ( axi_araddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] )
        4'h0:  // status_reg;
            begin
            end
        4'hD:  // ch01_data_reg;
            begin
              status_reg[1:0] <=  2'b0;
            end
        4'hE:  // ch23_data_reg;
            begin
              status_reg[3:2] <=  2'b0;
            end
        4'hF:  // ch45_data_reg;
            begin
              status_reg[5:4] <=  2'b0;
            end
        default:
            begin
              status_reg <= status_reg;
            end
      endcase
    end
    if ((sck_posedge == 1'b1) && (conv_internal == 1'b1 )) begin
      case (bit_counter)
        7'h10:
        begin
          ch01_data_reg[15: 0] <= {2'b00, input_data_reg[13:0]};
          status_reg[0]        <=  1'b1;
        end
        7'h20:
        begin
          ch01_data_reg[31:16] <= {2'b00, input_data_reg[13:0]};
          status_reg[1]        <=  1'b1;
        end
        7'h30:
        begin
          ch23_data_reg[15: 0] <= {2'b00, input_data_reg[13:0]};
          status_reg[2]        <=  1'b1;
        end
        7'h40:
        begin
          ch23_data_reg[31:16] <= {2'b00, input_data_reg[13:0]};
          status_reg[3]        <=  1'b1;
        end
        7'h50:
        begin
          ch45_data_reg[15: 0] <= {2'b00, input_data_reg[13:0]};
          status_reg[4]        <=  1'b1;
        end
        7'h60:
        begin
          ch45_data_reg[31:16] <= {2'b00, input_data_reg[13:0]};
          status_reg[5]        <=  1'b1;
        end
        default:
        begin
          ch01_data_reg <= ch01_data_reg;
          ch23_data_reg <= ch23_data_reg;
          ch45_data_reg <= ch45_data_reg;
        end
      endcase
    end
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    sclk_reg  <= 1'b0;
  end else begin
    sclk_reg  <= sclk;
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    bit_counter <= 7'd0;
  end else if (sck_posedge == 1'b1) begin
    if  (state_reg == READ_DATA_STATE) begin
      bit_counter <= bit_counter + 7'd1;
    end else begin
      bit_counter <= 7'd0;
    end
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    input_data_reg <= 16'd0;
  end else if (sck_posedge == 1'b1) begin
    if (state_reg == READ_DATA_STATE) begin
      input_data_reg <= {input_data_reg[15:0], sdo_reg};
    end else begin
      input_data_reg <= 16'd0;
    end
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    conv_internal <= 1'b0;
  end else if (sck_posedge == 1'b1) begin
    if ((state_next == GEN_CONV_STATE) && (state_reg == READ_DATA_STATE)) begin
      conv_internal <= 1'b1;
    end else if (state_reg == IDLE_STATE) begin
      conv_internal <= 1'b0;
    end
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    state_reg   <= 2'b00;
  end else if (sck_posedge == 1'b1) begin
    state_reg   <= state_next;
  end
end

always @(*)
begin
  state_next = state_reg;
  case (state_reg)
    IDLE_STATE:
    begin
      if ((tris_O == 1'b1) ) begin
        state_next = READ_DATA_STATE;
      end else if (start_conv == 1'b1) begin
        state_next = GEN_CONV_STATE;
      end
    end
    READ_DATA_STATE:
    begin
      if (bit_counter == BIT_NUM) begin
        if (conv_internal == 1'b1) begin
          state_next = IDLE_STATE;
        end else begin
          state_next = GEN_CONV_STATE;
        end
      end
    end
    GEN_CONV_STATE:
    begin
      state_next = READ_DATA_STATE;
    end
    default:
    begin
      state_next = IDLE_STATE;
    end
  endcase

end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    sck_enable  <= 1'b0;
  end else begin
    sck_enable <= 1'b1;
    /*
    if (tris_O == 1'b1) begin
      sck_enable <= 1'b1;
    end else if (state_next  == IDLE_STATE       &&
                 state_reg   == READ_DATA_STATE  &&
                 sck_posedge == 1'b1) begin
      sck_enable <= 1'b0;
    end
    */
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    sck_counter <= 2'b00;
  end else begin
    if (sck_enable == 1'b0) begin
      sck_counter <= 2'b00;
    end else begin
      sck_counter <= sck_counter + 2'b01;
    end
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
  end else begin
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin //
    counter <= 20'H0;
  end else begin
    if (tris_O_reg == 1'b1 && tris_O == 1'b0) begin
      counter <= 20'H0;
    end else begin
      counter <= counter + 20'b1;
    end
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    slv_reg1_reg  <= {C_S_AXI_DATA_WIDTH{1'b0}};
  end else if (sck_posedge == 1'b1) begin
    if (state_reg == IDLE_STATE) begin
      slv_reg1_reg  <= slv_reg1;
    end
  end
end

always @(posedge S_AXI_ACLK) begin
  if (S_AXI_ARESETN == 1'b0) begin
    tris_O_reg <= 1'b0;
  end else begin
    tris_O_reg <= tris_O;
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    sdo_reg <= 1'b0;
  end else begin
    sdo_reg <= sdo_i;
  end
end


always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
    slv_reg11[0] <= 1'b0;
  end else begin
    slv_reg11[0] <= 1'b0;
  end
end

 /////////////////////////////////////////
always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
  end else if (sck_posedge == 1'b1) begin
  end
end

always @(posedge S_AXI_ACLK)
begin
  if (S_AXI_ARESETN == 1'b0) begin
  end else begin
  end
end

// User logic ends

  endmodule

