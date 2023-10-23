`timescale 1ns / 1ps
module fir
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(   
    output  wire                     awready,
    output  wire                     wready, 
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr, 
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata, 
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

// Define state codes for the state machine
parameter[2:0]  IDLE       = 3'd0,
                WRITE      = 3'd1,
                COMPUTE    = 3'd2,
                ANSWER     = 3'd3,
                FINAL      = 3'd4;

localparam AP_CTRL   = 8'h00;
localparam AP_LENGTH = 8'h10;
localparam COEF_BASE = 8'h20;
localparam COEF_HIGH = 8'hff;

begin
// Define internal registers
    reg ap_start;
    reg ap_done;
    reg ap_idle;
    reg [3:0] ctr;
    reg ss_tready_r;
    reg [2:0] stt_data_n;
    reg [2:0] stt_data_c;
    reg signed [pDATA_WIDTH - 1 : 0] coef_lite [0 : 128];
    reg [31:0] rdata_r;
    reg arready_r;
    reg rvalid_r;
    reg awready_r=0;
    reg wready_r=0;
    reg [3:0]ctr_lite;
    wire [(pADDR_WIDTH-1):0] raddr;
    wire ar_high; 
    wire signed [4:0]index_coef_1;
    wire [4:0]index_coef_2;

    assign index_coef_1 = (stt_data_c == COMPUTE)? (ctr - ctr_lite):0;
    assign index_coef_2 = (index_coef_1 < 0)? (index_coef_1+11):index_coef_1;

    reg [(pDATA_WIDTH-1):0] result;
    reg tvalid_r;
    assign sm_tdata = result;
    assign sm_tvalid = tvalid_r;
    reg [9:0] len; 

    assign ss_tready = ss_tready_r;
    // Control signals
    assign ar_high = (arvalid & arready);
    assign arready = arready_r;
    assign rvalid = rvalid_r; 
    assign sm_tlast = (coef_lite[0]==2)?1:0;

    // BRAM for data 
    assign data_EN = (stt_data_c == WRITE  || stt_data_c == COMPUTE || stt_data_c == IDLE )? 1 : 0;
    assign data_Di = (ss_tready&&ss_tvalid)? ss_tdata : 0;
    assign data_WE = (ss_tready || stt_data_c == IDLE )?4'b1111:0;
    assign data_A = (data_EN && stt_data_c == IDLE )? ((ctr_lite-2)*4):
                    (data_EN && stt_data_c == WRITE )? (4 * ctr):
                    (data_EN && stt_data_c == COMPUTE)? (4 * ctr_lite):0;

    
    assign rdata = rdata_r;
    assign raddr = araddr;
    assign awready = awready_r;
    assign wready = wready_r;


    // BRAM for coefficients
    assign tap_EN = ((awready && awaddr>=COEF_BASE && awaddr<=COEF_HIGH) || stt_data_c == COMPUTE)?1:0;
    assign tap_Di = (wready)? wdata:0;
    assign tap_WE = (awvalid && stt_data_c == IDLE )?4'b1111:0;
    assign tap_A  = (tap_EN && stt_data_c == IDLE )?((ctr_lite-1)*4):  
                    (tap_EN && stt_data_c == COMPUTE)? ((index_coef_2)*4):0;

    // Initialize rdata_r
    integer i;
    always@(posedge axis_clk or negedge axis_rst_n)begin // rdata
        if (!axis_rst_n)begin
            for(i = 0 ; i < 32 ; i = i + 1)begin
                rdata_r[i] <= 0; 
            end
        end
        else begin 
            if(ar_high)begin
                case(raddr)         //看raddr的值等於AP_CTRL or AP_LENGTH
                    AP_CTRL:begin    
                        rdata_r <= {coef_lite[araddr+3][8:0],coef_lite[araddr+2][7:0],
                            coef_lite[araddr+1][7:0],coef_lite[araddr][7:0]};
                    end
                    AP_LENGTH:begin   
                        rdata_r <= {coef_lite[araddr+3][8:0],coef_lite[araddr+2][7:0],
                            coef_lite[araddr+1][7:0],coef_lite[araddr][7:0]};
                    end
                endcase
            end
            else if(arvalid)begin
                rdata_r <= {coef_lite[araddr+3][8:0],coef_lite[araddr+2][7:0],
                            coef_lite[araddr+1][7:0],coef_lite[araddr][7:0]};
            end  
        end      
    end

    // Operations on coef_lite array
    always @(posedge axis_clk or negedge axis_rst_n) begin //coef_lite
        if (!axis_rst_n)begin
            for(i = 0 ; i < 128 ; i = i + 1)begin
                if(i == 0)begin
                    coef_lite[0] <= 4; //ap_idle == 1
                end
                else begin
                    coef_lite[i] <= 0;
                end
            end
        end
        else  begin
            if(stt_data_n == ANSWER && len == 1)begin
                coef_lite[0]<= 2; //start == 0, ap_idle == 0
            end
            else if(stt_data_n != IDLE  && stt_data_n != FINAL)begin
                coef_lite[0]<= 0; //start == 0, ap_idle == 0
            end
            else if(stt_data_n == FINAL)begin
                coef_lite[0]<= 4; //start == 0, ap_idle == 0
            end
            
            else if (awvalid)begin  // Write data into coef_lite
                coef_lite[awaddr]    <= wdata[7:0];
                coef_lite[awaddr + 1]<= wdata[15:8];
                coef_lite[awaddr + 2]<= wdata[23:16];
                coef_lite[awaddr + 3]<= wdata[31:23];
            end
            else begin
                for(i = 0 ; i < 128 ; i = i + 1)begin
                    coef_lite[i] <= coef_lite[i];
            end
            end
        end
    end

    // Control signal blocks
    always@(posedge axis_clk or negedge axis_rst_n)begin //arready_r
        if(!axis_rst_n)begin
            arready_r <= 0;
        end
        else begin
            if(arready == 1)begin
                arready_r <= 0; 
            end
            else if(arvalid == 1 && rvalid==0)begin
                arready_r <= 1;
            end
        end
    end
    always@(posedge axis_clk or negedge axis_rst_n)begin //rvalid_r
        if(!axis_rst_n)begin
            rvalid_r <= 0;
        end
        else begin
            if(arready == 1)begin
                rvalid_r <= 1; 
            end
            else if(arready == 0 && rready)begin
                rvalid_r <= 0;
            end
        end
    end

    // Write control
    always@(posedge axis_clk or negedge axis_rst_n)begin //awready_r
        if(!axis_rst_n)begin
            awready_r <= 0;
        end
        else begin
            if(awready == 1)begin
                awready_r <= 0;
            end
            
            else if(awvalid == 1)begin
                awready_r <= 1; 
            end
        end
    end
    always@(posedge axis_clk or negedge axis_rst_n)begin //wready
        if(!axis_rst_n)begin
            wready_r <= 0;
        end
        else begin           
            if(wready == 1)begin
                wready_r <= 0;
            end 
            else if(wvalid == 1)begin
                wready_r <= 1;
            end
        end
    end
    always@(posedge axis_clk or negedge axis_rst_n)begin // ctr_lite
        if(!axis_rst_n)begin
            ctr_lite <= 0;
        end
        else begin
            if(stt_data_c == WRITE )begin
                ctr_lite <= 0;
            end
            else if(stt_data_c == COMPUTE)begin
                ctr_lite <= ctr_lite + 1;
            end
            else if(ctr_lite == 13)begin 
                ctr_lite <= 0;
            end
            else if(wready)begin
                ctr_lite <= ctr_lite + 1;
            end
        end
    end

    // Counter to move through the input data
    always@(posedge axis_clk or negedge axis_rst_n)begin 
        if(!axis_rst_n)begin
            ctr <= 0;
        end
        else begin
            if(ctr == 11)begin
                ctr <= 0;
            end
            else if(stt_data_n == ANSWER)begin 
                ctr <= ctr + 1;
            end
        end
    end

    // Toggle ss_tready_r when state is WRITE
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            ss_tready_r <= 0;
        end
        else begin
            if(stt_data_n == WRITE )begin 
                ss_tready_r <= 1;
            end
            else begin
                ss_tready_r <= 0;
            end
             
        end
    end

//STATE
    always@(*)begin
        case(stt_data_c)
            IDLE :begin 
                if(coef_lite[0][0]==1)begin
                    stt_data_n = WRITE ;
                end
                else begin
                    stt_data_n = IDLE ;
                end

            end
            WRITE :begin  // 1 cycle to write the data into BRAM
                    stt_data_n = COMPUTE;
            end
            COMPUTE:begin
                if(ctr_lite == 11)begin // 11 cycle to calculate the multiply & add
                    stt_data_n = ANSWER;
                end
                else begin
                    stt_data_n = COMPUTE;
                end
            end
            ANSWER:begin // 1 cycle to compare with the golden
                if(sm_tready && len != 1)begin // ready to output answer
                    stt_data_n = WRITE ;
                end
                else if(sm_tready && len == 1 && ss_tlast == 1)begin
                    stt_data_n = FINAL;
                end
                else begin
                    stt_data_n = ANSWER;
                end
            end
            FINAL:begin
                stt_data_n = FINAL;
            end
                
        endcase
    end

    // Update state
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            stt_data_c <= 0;
        end
        else begin
            stt_data_c <= stt_data_n;
        end
    end
    
    // FIR filter calculation
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            result <= 0;
        end
        else begin
            if(stt_data_c == COMPUTE && ctr_lite > 0)begin
                result <= result + data_Do * tap_Do;
            end
            else if(stt_data_c == WRITE )begin
                result <= 0;
            end
        end
    end

    // Set tvalid_r signal
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            tvalid_r <= 0;
        end
        else begin
            if(stt_data_n == ANSWER)begin
                tvalid_r <= 1;
            end
            else begin
                tvalid_r <= 0;
            end
        end
    end

    // Compute len signal
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            len <= 0;
        end
        else begin
            if(stt_data_c == IDLE )begin
                len <= {coef_lite[19][8:0],coef_lite[18][7:0],
                            coef_lite[17][7:0],coef_lite[16][7:0]};
            end
            else if(stt_data_c == ANSWER)begin
                len <= len - 1;
            end
        end
    end
end
endmodule