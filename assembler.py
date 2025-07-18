def signed(input):
    if input<0:
        return 256 + input
    else:
        return input


filename = "program.txt"
outputfile = "program.mem"


file = open(filename)

JMPopcodes =  { 

    "JMP" : 0b00110,
    "JEQ" : 0b10010,
    "JNE" : 0b10011,
    "JCC" : 0b10100,
    "JCS" : 0b10101, 
    "JMI" : 0b10110, 
    "JPL" : 0b10111, 
    "JGE" : 0b11000, 
    "JLT" : 0b11001,
    "JGT" : 0b11010, 
    "JLE" : 0b11011

}

ALUopcodes = {

    "MOV" : 0b0000,
    "ADD" : 0b0001,
    "SUB" : 0b0010,
    "ADC" : 0b0110,
    "SBC" : 0b0111

}

MEMopcodes = {

    "LDR" : 0b0100,
    "STR" : 0b0101

}

STACKopcodes = {

    "PUSH" : 0b11100,
    "POP" : 0b11110

}

SUBROUTINEopcodes = {

    "CALL" : 0b11101,
    "RET" : 0b11111

}

machinevector = []

assemblerprogram = []

linecnt = 0
for line in file:
    assemblerprogram.append(line)
    linecnt += 1

for line in assemblerprogram:

    words = line.split()
    machineline = ""
    wordcnt = len(words)

    try:

        if words[0] in ALUopcodes:
            machineline += format(ALUopcodes[words[0]], '04b')
            instype = "ALU"

        elif words[0] in MEMopcodes:
            machineline += format(MEMopcodes[words[0]], '04b')
            instype = "MEM"

        elif words[0] in JMPopcodes:
            machineline += format(JMPopcodes[words[0]], '05b')
            instype = "JMP"

        elif words[0] in STACKopcodes:
            machineline += format(STACKopcodes[words[0]], '05b')
            instype = "STACK"

        elif words[0] in SUBROUTINEopcodes:
            machineline += format(SUBROUTINEopcodes[words[0]], '05b')
            instype = "SUBROUTINE"

        elif words[0] == "CMP":
            machineline += "1000"
            instype = "CMP"
            

    except Exception as syntax:
        print ("Syntax Error")

    if ("#" in line and instype != "JMP" and wordcnt == 3) or ("#" in line and instype == "CMP"): 
        machineline += "1"
        machineline += format(int(words[1][1]), '03b')
        machineline += format(signed(int(words[2][1:])), '08b')

    elif "#" not in line and instype == "ALU" and wordcnt == 4:
        machineline += "0"
        machineline += format(int(words[1][1]), '03b')
        machineline += format(int(words[2][1]), '03b')
        machineline += format(int(words[3][1]), '03b')
        machineline += "00"

    elif "#" not in line and words[0] == "MOV":
        machineline += "0"
        machineline += format(int(words[1][1]), '03b')
        machineline += format(int(words[2][1]), '03b')
        machineline += "00000"

    elif instype == "JMP" and wordcnt == 2:
        machineline += "000"
        machineline += format(signed(int(words[1][1:])), '08b')

    elif "#" not in line and instype == "MEM" and wordcnt == 3:
        machineline += "0"
        machineline += format(int(words[1][1]), '03b')
        machineline += "000"
        machineline += format(int(words[2][1]), '03b')
        machineline += "00"

    elif "#" not in line and instype == "MEM" and wordcnt == 3:
        machineline += "0"
        machineline += format(int(words[1][1]), '03b')
        machineline += "000"
        machineline += format(int(words[2][1]), '03b')
        machineline += "00"

    elif "#" not in line and instype == "CMP":
        machineline += "0000"
        machineline += format(int(words[1][1]), '03b')
        machineline += format(int(words[2][1]), '03b')
        machineline += "00"

    elif instype == "STACK":
        machineline += format(int(words[1][1]), '03b')
        machineline += "00000000"

    elif instype == "SUBROUTINE":
        machineline += "000"
        if words[0] == "CALL":
            machineline += format(int(words[1][1:]), '08b')
        else:
            machineline += "00000000"

    else:
        print ("Syntax Error")

    
    machinevector.append(machineline)


out = open(outputfile, 'w')

for line in machinevector:
    out.write(line + "\n") 

out.close()
file.close()