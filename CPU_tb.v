`timescale 1ns/1ps

module datapath_tb; // creating test bench module (arbitrarily called datapath). Many of the design modules
                    // can be instantiated and tested in here.

    reg clk;
    wire[7:0] r0, r1, r2, r3, r4, r5, r6, r7;

    // note:
    // reg for inputs, wire for outputs


    CPU uut(// instantiating the actual design module inside
                  // its worth nothing 'uut' is just the name of the instance of the module being tested
                  // uut stands for 'unit under test'

        .clk(clk), .r0(r0), .r1(r1), .r2(r2), .r3(r3), .r4(r4), .r5(r5),
        .r6(r6), .r7(r7)

    );


    initial clk = 0;
    always #5 clk = ~clk;

    initial begin

        $dumpfile("datapath_tb.vcd"); // sets up the VCD file for waveform simulation
        $dumpvars(0, datapath_tb); // must be at top of begin without any delays


        /*
        #10 ins = 16'h0805; // r0 = 5
        #10 ins = 16'h0100; // r1 = r0
        #10 ins = 16'h1220; // r2 = r0 + r1
        #10 ins = 16'h2A02; // r2 = r2 - 2 THIS IS THE PROBLEM, IMM IS 1 OPERAND
        #10 ins = 16'h5210; // str r2 #16 THIS IS STORING THE REGISTER, 
        #10 ins = 16'h4310; // ldr r3 #16
        */



        #300;
        $finish;

    end

endmodule

