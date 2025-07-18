// decode is responsible for segmenting and processing 16 bit instruction
module decode (

    input wire[15:0] ins,
    output wire[3:0] opcode,
    output wire[7:0] imm8,
    output wire[2:0] ra_addr, rb_addr, rc_addr,
    output wire imm_en, WEN, data_WEN

);

    reg WEN_reg, data_WEN_reg;

    assign opcode = ins[15:12];
    assign imm8 = ins[7:0];
    assign ra_addr = ins[10:8];
    assign rb_addr = ins[7:5];
    assign rc_addr = ins[4:2];
    assign imm_en = ins[11];
    
    always @(*) begin

        if(opcode == 4'h0 || opcode == 4'h1 || opcode == 4'h2 || opcode == 4'h4 || opcode == 4'h6 || opcode == 4'h7 || opcode == 4'hF && ~imm_en) begin
            WEN_reg = 1;
        end

        else begin
            WEN_reg = 0;
        end

        if(opcode == 4'b0101 || opcode == 4'hE) begin
            data_WEN_reg = 1;
        end

        else begin
            data_WEN_reg = 0;
        end

    end

    assign WEN = WEN_reg;
    assign data_WEN = data_WEN_reg;

endmodule


// full adder
module fadd (

    input wire a, b, cin,
    output wire sum, cout

);

    assign sum = a^b^cin;
    assign cout = (a&b)|(a&cin)|(b&cin);

endmodule


// 8 bit ripple carry adder
module addsub8 (

    input wire[7:0] a, b,
    input wire sub, cin,
    output wire[7:0] sum,
    output wire cout, cin_msb

);

    genvar i;

    wire[7:0] carry;

    generate

        fadd fa0 ( .a(a[0]), .b(b[0]), .cin(cin), .cout(carry[0]), .sum(sum[0]) );
        for(i=1; i<8; i = i + 1) begin: loop
            fadd fa ( .a(a[i]), .b(b[i]), .cin(carry[i-1]), .cout(carry[i]), .sum(sum[i]) );
        end

    endgenerate

    assign cout = carry[7];
    assign cin_msb = carry[6];

endmodule


// 16 bit ripple carry adder
module addsub16 (

    input wire[15:0] a, b,
    output wire[15:0] sum

);

    wire[15:0] carry;

    genvar i;

    generate

        fadd fa0 ( .a(a[0]), .b(b[0]), .cin(1'b0), .cout(carry[0]), .sum(sum[0]) );
        for(i = 1; i<16; i=i+1) begin: loop
            fadd fa ( .a(a[i]), .b(b[i]), .cin(carry[i-1]), .cout(carry[i]), .sum(sum[i]) );
        end

    endgenerate

endmodule


// 8 bit width stack pointer register
module SP (

    input wire WEN, clk, reset,
    input wire[7:0] din,
    output wire[7:0] dout

);

    reg[7:0] SP;

    always @(posedge clk) begin

        if(reset) begin
            SP <= 8'hFF;
        end

        else if(WEN) begin
            SP <= din;
        end

    end

    assign dout = SP;

endmodule


// arithmetic logic unit
module ALU (

    input wire[3:0] opcode,
    input wire[7:0] rb, rc, imm8,
    input wire imm_en, flagc_in,
    output wire[7:0] ALU_out,
    output wire flagc_out, cin_msb

);

    wire[7:0] adder_out;
    wire sub, carry_out;

    reg[7:0] addsub_reg, addsub_in, out_reg, pass_through;
    reg carry_in;

    // opcodes using subtraction
    assign sub = (opcode == 4'h2 || opcode == 4'h7 || opcode == 4'h8 || opcode == 4'hE);

    always @(*) begin

        if(opcode == 4'h5) begin
            pass_through = rb;
        end

        // imm_en used to choose push/call and pop/ret for these opcodes
        else if(imm_en && opcode != 4'hE && opcode != 4'hF) begin
            pass_through = imm8;
            addsub_reg = imm8;
        end
        
        else begin
            pass_through = rb;
            addsub_reg = rc;
        end

        // opcodes that require adder
        if(opcode == 4'h1 || opcode == 4'h2 || opcode == 4'h6 || opcode == 4'h7 || opcode == 4'h8 || opcode == 4'hE || opcode == 4'hF) begin
            out_reg = adder_out; 
        end

        else begin
            out_reg = pass_through;
        end

        // opcodes using subtraction without carry
        if(opcode == 4'h2 || opcode == 4'h8 || opcode == 4'hE) begin
            carry_in = 1'b1;
        end

        else if(opcode == 4'h1) begin
            carry_in = 1'b0;
        end
        
        // opcodes featuring carry in
        else if(opcode == 4'h6 || opcode == 4'h7) begin
            carry_in = flagc_in;
        end

        else begin
            carry_in = 1'b0;
        end

        if(sub) begin
            addsub_in = ~addsub_reg;
        end

        else begin
            addsub_in = addsub_reg;
        end

    end

    assign ALU_out = out_reg;
    assign flagc_out = (out_reg == adder_out) ? carry_out : 1'b0;

    addsub8 ALU_adder ( .a(rb), .b(addsub_in), .sub(sub), .sum(adder_out), .cout(carry_out), .cin(carry_in), .cin_msb(cin_msb) );

endmodule


// 8 bit width register file 
module regfile (

    input wire WEN, clk,
    input wire[7:0] din1,
    input wire[2:0] addr1, addr2, addr3,
    output wire[7:0] dout2, dout3, r0, r1, r2, r3, r4, r5, r6, r7

);

    reg[7:0] registers[7:0];

    always @(posedge clk) begin

        if(WEN) begin
            registers[addr1] <= din1;
        end

    end

    assign dout2 = registers[addr2];
    assign dout3 = registers[addr3];

    assign r0 = registers[0];
    assign r1 = registers[1];
    assign r2 = registers[2];
    assign r3 = registers[3];
    assign r4 = registers[4];
    assign r5 = registers[5];
    assign r6 = registers[6];
    assign r7 = registers[7];

endmodule


// datapath responsible for processing data
module datapath (

    input wire[15:0] ins,
    input wire[7:0] data_in, pc_str,
    input wire clk, reset, state,
    output wire[7:0] ALU_out, r0, r1, r2, r3, r4, r5, r6, r7, addr_out, data_out, SP_value, ALUinA, ALUinB,
    output wire data_WEN, flagc_out, flagv_out, flagn_out, flagz_out

);  

    integer i;

    wire[7:0] imm8, rb, rc, din1;
    wire[3:0] opcode;
    wire[2:0] ra_addr, rb_addr, rc_addr, regfile_addr2;
    wire imm_en, WEN, SP_WEN;

    reg[7:0] addr_out_reg, data_out_reg, ALUinA_reg, ALUinB_reg;
    reg[2:0] regfile_addr2_reg;
    
    assign SP_WEN = (opcode == 4'hE || opcode == 4'hF);

    wire flagcd;
    reg flagc;
    wire cin_msb;
    
    wire flagvd;
    reg flagv;

    wire flagnd;
    reg flagn;

    wire flagzd;
    reg flagz;

    decode decoder (
         .ins(ins), .opcode(opcode), .imm8(imm8), .ra_addr(ra_addr), .rb_addr(rb_addr), .rc_addr(rc_addr), .WEN(WEN), .imm_en(imm_en),
         .data_WEN(data_WEN) 
    );

    regfile register_file ( 
        .WEN(WEN), .clk(clk), .din1(din1), .dout2(rb), .dout3(rc), .addr1(ra_addr), .addr2(regfile_addr2), .addr3(rc_addr),
        .r0(r0), .r1(r1), .r2(r2), .r3(r3), .r4(r4), .r5(r5), .r6(r6), .r7(r7)  
    );

    ALU ALU_dpath ( 
        .opcode(opcode), .imm8(imm8), .rb(ALUinA), .rc(ALUinB), .imm_en(imm_en), .ALU_out(ALU_out), .flagc_in(flagc), .flagc_out(flagcd),
        .cin_msb(cin_msb)
    );

    SP stack_pointer (
        .WEN(SP_WEN), .clk(clk), .din(ALU_out), .dout(SP_value), .reset(reset)
    );

    assign din1 = (opcode == 4'b0100 || opcode == 4'hF) ? data_in : ALU_out;

    always @(*) begin

        // instructions with two operands require read and write from same register ra
        if(imm_en || opcode == 4'd5 || opcode == 4'hE || opcode == 4'hF) begin
            regfile_addr2_reg = ra_addr;
        end
        
        else begin 
            regfile_addr2_reg = rb_addr;
        end

        // call instruction requires pc to be saved & imm_en used as part of opcode
        if(opcode == 4'hE && imm_en) begin
            data_out_reg = pc_str; 
        end

        else if(imm_en) begin
            data_out_reg = ALU_out;  
        end

        else begin
            data_out_reg = rb;
        end

        // reister load and store instructions use rc as address
        if(opcode == 4'h4 && ~imm_en || opcode == 4'h5 && ~imm_en) begin
            addr_out_reg = rc;
        end

        // stack pointer address saving
        else if(opcode == 4'hE) begin
            addr_out_reg = ALU_out;
        end 

        // stack pointer address retrieving
        else if(opcode == 4'hF) begin
            addr_out_reg = SP_value;
        end 

        else begin
            addr_out_reg = imm8;
        end

        // stack pointer computed by ALU
        if(opcode == 4'hE || opcode == 4'hF) begin
            ALUinA_reg = SP_value;
            ALUinB_reg = 8'd1;
        end

        else begin
            ALUinA_reg = rb;
            ALUinB_reg = rc;
        end

    end

    // assigning reg to wires
    assign regfile_addr2 = regfile_addr2_reg;  
    assign addr_out = addr_out_reg;
    assign data_out = data_out_reg;
    assign ALUinA = ALUinA_reg;
    assign ALUinB = ALUinB_reg;

    // assigning flag outputs
    assign flagc_out = flagc;
    assign flagv_out = flagv;
    assign flagn_out = flagn;
    assign flagz_out = flagz;

    // computing flags
    assign flagvd = flagcd ^ cin_msb;
    assign flagnd = ALU_out[7];
    assign flagzd = (ALU_out == 8'h00);

    always @(posedge clk) begin

        // updating flags
        flagc = flagcd;
        flagv = flagvd;
        flagn = flagnd;
        flagz = flagzd;

    end

endmodule


// conditions for immediate offset jumps
module COND (

    input wire[15:0] imm_ext,
    input wire[3:0] opcode,
    input wire jmp_cond, n, z, c, v,
    output wire[15:0] pc_offset

);

    reg[15:0] offset_wire;
    reg ncond, zcond, ccond, vcond, jge_cond, jgt_cond;

    always @(*) begin

        // jump condition bit inverts jump condition
        if(jmp_cond) begin
            ncond = ~n;
            zcond = ~z;
            ccond = ~c;
            vcond = ~v;
            jge_cond = n^v;
            jgt_cond = (n^v)||z;
        end

        else begin
            ncond = n;
            zcond = z;
            ccond = c;
            vcond = v;
            jge_cond = ~(n^v);
            jgt_cond = ~(n^v)&&~z;
        end

        // jump offset for each jump opcode and condition

        if(opcode == 4'b0011) begin
            offset_wire = imm_ext;
        end

        else if(opcode == 4'b1001 && zcond) begin
            offset_wire = imm_ext;
        end

        else if(opcode == 4'b1010 && ccond) begin
            offset_wire = imm_ext;
        end

        else if(opcode == 4'b1011 && ncond) begin
            offset_wire = imm_ext;
        end

        else if(opcode == 4'b1100 && (jge_cond)) begin
            offset_wire = imm_ext;
        end

        else if(opcode == 4'b1101 && (jgt_cond))begin
            offset_wire = imm_ext;
        end

        else if(opcode == 4'hE && jmp_cond || opcode == 4'hF && jmp_cond) begin
            offset_wire = 16'd1;
        end

        else begin
            offset_wire = 16'd1;
        end

    end

    assign pc_offset = offset_wire;

endmodule


// state machine used for saving program counter during subroutines
module state_machine (

    input wire reset, en, init, clk,
    output wire state, done

);

    wire dff_in;

    reg Q, done_reg;

    always @(posedge clk) begin

        // resets when reset is high or when toggle is done
        if(reset || done) begin
            Q <= 0;
            done_reg = 0;
        end

        else if(en) begin
            Q <= dff_in;
        end

    end

    always @(*) begin

        // state machine must toggle just once
        done_reg = (Q == ~init);

    end

    // state machine toggles from initial value
    assign dff_in = ~Q;
    assign state = Q ? ~init : init;
    assign done = done_reg;

endmodule


// control path is responsible for the program counter
module controlpath (

    input wire[15:0] ins_in,
    input wire[7:0] data_in,
    input wire clk, flagc, flagv, flagn, flagz, reset,
    output wire[15:0] ins_out, pc_next,
    output reg[15:0] pc,
    output wire[7:0] pc_str,
    output wire pc_en_out, state_machine_en, init, state, done

);

    wire[15:0] pc_offset, pc_adder_out;
    wire[3:0] opcode;
    wire imm_en;

    reg[15:0] imm_ext, pc_RET, pc_next_reg;
    reg[7:0] pc_str_reg;
    reg init_reg, state_machine_en_reg, pc_en;

    integer i;

    assign opcode = ins_in[15:12];
    assign imm_en = ins_in[11];

    always @(*) begin

        // sign extension of immediate operand
        imm_ext = ins_in[7:0];
        for(i = 8; i<16; i=i+1) begin
            imm_ext[i] = imm_ext[7];
        end

        // state machine enabled for call/ret instructions
        if(opcode == 4'hE && imm_en || opcode == 4'hF && imm_en) begin
            state_machine_en_reg = 1;
        end

        else begin
            state_machine_en_reg = 0;
        end

        // initial value 0 when calling subroutine to push PC
        if(opcode == 4'hE && imm_en) begin
            init_reg = 0;
            pc_next_reg = imm_ext;
        end

        // initial value 1 when returning from subroutine to pop PC
        else if(opcode == 4'hF && imm_en) begin
            pc_next_reg = pc_RET;
            init_reg = 1;
        end

        else begin 
            pc_next_reg = pc_adder_out;
            init_reg = 1;
        end

        // with subroutine instructions, state machine state is used to select upper or lower bytes

        if(opcode == 4'hE && imm_en && ~state) begin
            pc_str_reg = pc_adder_out[7:0];
        end

        else if(opcode == 4'hE && imm_en && state) begin  
            pc_str_reg = pc_adder_out[15:8];
        end

        if(opcode == 4'hF && imm_en && state) begin
            pc_RET[15:8] = data_in;
        end

        else if(opcode == 4'hF && imm_en && ~state) begin
            pc_RET[7:0] = data_in;
        end

        // pc cannot update until pc is saved / retrieved

        if(opcode == 4'hE && imm_en && ~done) begin
            pc_en = 0;
        end

        else if(opcode == 4'hF && imm_en && ~done) begin
            pc_en = 0;
        end

        else begin
            pc_en = 1;
        end

    end

    always @(posedge clk) begin

        if(pc_en) begin
            pc <= pc_next;
        end

        else if(reset) begin
            pc <= 0;
        end

    end

    COND conditions ( .imm_ext(imm_ext), .opcode(opcode), .jmp_cond(ins_in[11]), .c(flagc), .v(flagv), .n(flagn), .z(flagz), 
    .pc_offset(pc_offset) );

    addsub16 counter( .a(pc), .b(pc_offset), .sum(pc_adder_out) );

    state_machine subroutine ( .reset(reset), .en(state_machine_en), .init(init), .state(state), 
    .done(done), .clk(clk) );


    assign ins_out = ins_in;

    // assigning reg to wires
    assign init = init_reg;
    assign pc_str = pc_str_reg;
    assign state_machine_en = state_machine_en_reg;
    assign pc_next = pc_next_reg;
    assign pc_en_out = pc_en;

endmodule


// memory for instructions
module instruction_rom (

    input wire[15:0] pc,
    output wire[15:0] ins

);

    reg[15:0] memory[0:65535];

    initial begin
        $readmemb("program.mem", memory);
    end

    assign ins = memory[pc];    

endmodule


// memory for data
module data_ram (

    input wire[7:0] addr, din,
    input wire en, clk,
    output wire[7:0] data

);

    reg[7:0] memory[0:255];

    always @(posedge clk) begin

        if(en) begin
            memory[addr] <= din;
        end

    end

    assign data = memory[addr];

endmodule


// full CPU module
module CPU (

    input wire clk, reset,
    output wire[7:0] r0, r1, r2, r3, r4, r5, r6, r7, ALU_out, ALUinA, ALUinB,
    output wire flagc, flagv, flagn, flagz, data_WEN, pc_en_out, init, state_machine_en, done, state,
    output wire[15:0] pc, pc_next,
    output wire[7:0] datapath_out, addr, datapath_in, SP_value, pc_str

);

    wire[15:0] ins_cpath, ins_dpath;

    instruction_rom codemem ( .pc(pc), .ins(ins_cpath) );

    controlpath c ( .ins_in(ins_cpath), .ins_out(ins_dpath), .pc(pc), .clk(clk), .flagc(flagc), .flagv(flagv), 
    .flagn(flagn), .flagz(flagz), .pc_str(pc_str), .data_in(datapath_in), .pc_next(pc_next), .pc_en_out(pc_en_out),
    .init(init), .state_machine_en(state_machine_en), .done(done), .state(state), .reset(reset) );

    data_ram datamem ( .en(data_WEN), .din(datapath_out), .addr(addr), .clk(clk), .data(datapath_in) );

    datapath d ( 

    .ins(ins_dpath), .clk(clk), .data_out(datapath_out), .data_WEN(data_WEN), .addr_out(addr), 
    .data_in(datapath_in), .r0(r0), .r1(r1), .r2(r2), .r3(r3), .r4(r4), .r5(r5), .r6(r6), .r7(r7), .flagc_out(flagc),
    .ALU_out(ALU_out), .flagv_out(flagv), .flagn_out(flagn), .flagz_out(flagz), .reset(reset), .SP_value(SP_value),
    .pc_str(pc_str), .state(state), .ALUinA(ALUinA), .ALUinB(ALUinB)

    );

endmodule