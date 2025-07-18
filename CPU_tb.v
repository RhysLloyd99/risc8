`timescale 1ns/1ps

module CPU_tb;

    reg clk, reset;
    wire[7:0] r0, r1, r2, r3, r4, r5, r6, r7, ALU_out, datapath_in, datapath_out, addr, SP_value, ALUinA, ALUinB, pc_str;
    wire flagc, flagv, flagn, flagz, data_WEN, pc_en_out, init, state_machine_en, done, state;
    wire[15:0] pc, pc_next;

    CPU uut(

        .clk(clk), .r0(r0), .r1(r1), .r2(r2), .r3(r3), .r4(r4), .r5(r5),
        .r6(r6), .r7(r7), .flagc(flagc), .flagv(flagv), .flagn(flagn), .flagz(flagz), 
        .ALU_out(ALU_out), .pc(pc), .reset(reset), .datapath_in(datapath_in), .datapath_out(datapath_out), 
        .addr(addr), .data_WEN(data_WEN), .SP_value(SP_value), .pc_next(pc_next), .pc_en_out(pc_en_out),
        .init(init), .state_machine_en(state_machine_en), .done(done), .state(state), .ALUinA(ALUinA), .ALUinB(ALUinB),
        .pc_str(pc_str)

    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin

        $dumpfile("CPU.vcd");
        $dumpvars(0, CPU_tb);

        reset = 1;
        #10 reset = 0;

        #3000;
        $finish;

    end

endmodule