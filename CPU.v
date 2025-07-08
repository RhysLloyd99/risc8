module decode(

    input wire[15:0] ins,
    output wire[3:0] opcode,
    output wire[7:0] imm8,
    output wire[2:0] ra, rb, rc,
    output wire immen, WEN, data_WEN

);

    reg WENalways;
    reg dataWENalways;

    assign opcode = ins[15:12];
    assign imm8 = ins[7:0];
    assign ra = ins[10:8];
    assign rb = ins[7:5];
    assign rc = ins[4:2];
    assign immen = ins[11];
    
    always @(*) begin

        if(ins[15:12] == 4'h0 || ins[15:12] == 4'h1 || ins[15:12] == 4'h2 || ins[15:12] == 4'h4) begin
            WENalways = 1;
        end

        else begin
            WENalways = 0;
        end

        if(opcode == 4'b0101) begin
            dataWENalways = 1;
        end

        else begin
            dataWENalways = 0;
        end

    end

    assign WEN = WENalways;
    assign data_WEN = dataWENalways;

endmodule



module fadd (

    input wire a, b, cin,
    output wire sum, cout

);

    assign sum = a^b^cin;
    assign cout = (a&b)|(a&cin)|(b&cin);

endmodule



module addsub8 (

    input wire[7:0] a, b,
    input wire sub,
    output wire[7:0] sum

);

    genvar i;

    wire[7:0] carry;
    wire[7:0] binv;
    wire cin;
    
    assign binv = sub ? ~b : b;
    assign cin = sub ? 1'b1 : 1'b0;

    generate

        fadd fa ( .a(a[0]), .b(binv[0]), .cin(cin), .cout(carry[0]), .sum(sum[0]) );
        for(i=1; i<8; i = i + 1) begin
            fadd fa ( .a(a[i]), .b(binv[i]), .cin(carry[i-1]), .cout(carry[i]), .sum(sum[i]) );
        end

    endgenerate

endmodule


module addsub16 (

    input wire[15:0] a, b,
    output wire[15:0] sum

);

    wire[15:0] carry;

    genvar i;

    generate

        fadd fa ( .a(a[0]), .b(b[0]), .cin(1'b0), .cout(carry[0]), .sum(sum[0]) );
        for(i = 1; i<16; i=i+1) begin
            fadd fa ( .a(a[i]), .b(b[i]), .cin(carry[i-1]), .cout(carry[i]), .sum(sum[i]) );
        end

    endgenerate

endmodule


module ALU(

    input wire[3:0] opcode,
    input wire[7:0] rb, rc, imm8,
    input wire immen,
    output wire[7:0] out

);

    wire[7:0] addsubin;
    wire sub;   
    wire outsel;
    wire[7:0] adderout;
    reg[7:0] outalways;
    reg[7:0] imm_or_not;

    assign addsubin = immen ? imm8 : rc;
    assign sub = (opcode == 4'h2);

    addsub8 instance1 ( .a(rb), .b(addsubin), .sub(sub), .sum(adderout) );

    always @(*) begin

        $display("immen=%b, imm8=%h, rc=%h, addsubin=%h, opcode=%h", immen, imm8, rc, addsubin, opcode);

        if(immen == 1) begin
            imm_or_not = imm8;
        end
        
        else begin
            imm_or_not = rb;
        end

        if(opcode == 4'h1 || opcode == 4'h2) begin
            outalways = adderout;
        end

        else begin
            outalways = imm_or_not;
        end

    end

    assign out = outalways;

endmodule



module regfile (

    input wire WEN, clk,
    input wire[7:0] din1,
    input wire[2:0] ad1, ad2, ad3,
    output wire[7:0] dout2, dout3, r0, r1, r2, r3, r4, r5, r6, r7

);

    reg[7:0] registers[7:0];

    always @(posedge clk) begin

        if(WEN == 1) begin
            registers[ad1] <= din1;
        end

    end

    assign dout2 = registers[ad2];
    assign dout3 = registers[ad3];

    assign r0 = registers[0];
    assign r1 = registers[1];
    assign r2 = registers[2];
    assign r3 = registers[3];
    assign r4 = registers[4];
    assign r5 = registers[5];
    assign r6 = registers[6];
    assign r7 = registers[7];


endmodule



module datapath (

    input wire[15:0] ins,
    input wire[7:0] data_in,
    input wire clk,
    output wire[7:0] ALUout, r0, r1, r2, r3, r4, r5, r6, r7, imm_out,
    output wire data_WEN

);  

    wire[3:0] opcode;
    wire[7:0] imm8;
    wire[2:0] ra_adr;
    wire[2:0] rb_adr; 
    wire[2:0] rc_adr;
    wire[7:0] rb;
    wire[7:0] rc;
    wire immen;
    wire WEN;
    wire[7:0] din1;
    reg[2:0] imm_adr;
    wire[2:0] imm_adr_wire;
    

    decode instance3 (
         .ins(ins), .opcode(opcode), .imm8(imm8), .ra(ra_adr), .rb(rb_adr), .rc(rc_adr), .WEN(WEN), .immen(immen),
         .data_WEN(data_WEN) 
    );

    regfile instance2 ( 
        .WEN(WEN), .clk(clk), .din1(din1), .dout2(rb), .dout3(rc), .ad1(ra_adr), .ad2(imm_adr_wire), .ad3(rc_adr),
        .r0(r0), .r1(r1), .r2(r2), .r3(r3), .r4(r4), .r5(r5), .r6(r6), .r7(r7)  
    );

    ALU instance4 ( 
        .opcode(opcode), .imm8(imm8), .rb(rb), .rc(rc), .immen(immen), .out(ALUout)
    );

    assign imm_out = imm8;
    assign din1 = (opcode == 4'b0100) ? data_in : ALUout;

    always @(*) begin

        if(immen == 1 || opcode == 4'h4 || opcode == 4'h5) begin
            imm_adr = ra_adr;
        end
        
        else begin 
            imm_adr = rb_adr;
        end

    end

    assign imm_adr_wire = imm_adr;

endmodule


module controlpath (

    input wire[15:0] ins_in,
    input wire clk,
    output wire[15:0] ins_out,
    output reg[15:0] pc


);

    wire[3:0] opcode;
    reg[15:0] imm_ext;
    reg[15:0] add_b;
    wire[15:0] pc_ff;

    integer i;

    addsub16 counter( .a(pc), .b(add_b), .sum(pc_ff) );

    assign opcode = ins_in[15:12];

    always @(*) begin

        imm_ext = ins_in[7:0];
        
        for(i = 8; i<16; i=i+1) begin
            imm_ext[i] = imm_ext[7];
        end

        if(opcode == 4'b0011) begin
            add_b = imm_ext;
        end

        else begin 
            add_b = 16'd1;
        end

    end

    always @(posedge clk) begin

        pc <= pc_ff;

    end

    assign ins_out = ins_in;

    initial pc = 16'd0;

endmodule


module instruction_rom (

    input wire[15:0] pc,
    output wire[15:0] ins

);

    reg[15:0] memory[0:255];

    initial begin
        $readmemh("program.mem", memory);
    end

    assign ins = memory[pc];    

endmodule


module data_ram (

    input wire[7:0] addr, din,
    input wire en, clk,
    output wire[7:0] data

);

    reg[7:0] memory[0:255];

    always @(posedge clk) begin

        if(en == 1) begin
            memory[addr] = din;
        end

    end

    assign data = memory[addr];

endmodule


module CPU(

    input wire clk,
    output wire[7:0] r0, r1, r2, r3, r4, r5, r6, r7

);

    wire[15:0] ins_c, ins_d, pc;
    wire[7:0] datapath_out, addr, datapath_in;
    wire data_WEN;


    instruction_rom codemem ( .pc(pc), .ins(ins_c) );
    controlpath c ( .ins_in(ins_c), .ins_out(ins_d), .pc(pc), .clk(clk) );
    data_ram datamem ( .en(data_WEN), .din(datapath_out), .addr(addr), .clk(clk), .data(datapath_in) );

    datapath d ( 

    .ins(ins_d), .clk(clk), .ALUout(datapath_out), .data_WEN(data_WEN), .imm_out(addr), 
    .data_in(datapath_in), .r0(r0), .r1(r1), .r2(r2), .r3(r3), .r4(r4), .r5(r5), .r6(r6), .r7(r7)

    );


endmodule


// some notes:

    // be careful about having wires that are connected in a sense to 3 places.
    // e.g. a wire thats connecting 2 components and whos data is assigned to another wire.
    // in principle this isn't incorrect, but causes simulation problems.

    // also, keep in mind wires for inputs in module initialisation, reg or wire for outputs, depending on application.
    // reg for always blocks, wires for assigning.

    // remember immediate instructions are 1 register operand

    // also, immen = 0 for memory instructions