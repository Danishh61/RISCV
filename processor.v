module processor( clk, reset, Result  );

    input clk, reset;
    output [31:0] Result;
    
    wire reg_write, mem2reg, alu_src, mem_write, mem_read;
    wire [3:0] alu_cc;
    wire [6:0] opcode, funct7;
    wire [2:0] funct3;
    wire [1:0] alu_op;
    
    data_path dp1(  .clk(clk),
                    .reset(reset),
                    .reg_write(reg_write),
                    .mem2reg(mem2reg),
                    .alu_src(alu_src),
                    .mem_write(mem_write),
                    .mem_read(mem_read),
                    .alu_cc(alu_cc),
                    .opcode(opcode),
                    .funct3(funct3),
                    .funct7(funct7),
                    .alu_result(Result) );
                    
     Controller c1( .Opcode(opcode),
                    .ALUSrc(alu_src),
                    .MemtoReg(mem2reg),
                    .MemWrite(mem_write),
                    .MemRead(mem_read),
                    .RegWrite(reg_write),
                    .ALUOp(alu_op) );
                    
     ALUController ac1( .ALUOp(alu_op),
                        .Funct3(funct3),
                        .Funct7(funct7),
                        .Operation(alu_cc) );
endmodule

module data_path #( 
    parameter PC_W = 8,         // Program Counter
    parameter IMS_W = 32,       // Instruction Width
    parameter RG_ADDRESS = 5,   // Register File Address
    parameter DATA_W = 32,      // Data Write Data
    parameter DM_ADDRESS = 9,   // Data Memory Address
    parameter ALU_CC_W = 4      // ALU Control Code Width
  )(
    // Initialize input, output signals
    input                clk,       // CLK in Datapath module
    input                reset,     // Reset in Datapath module
    input                reg_write, // RegWrite in Datapath module
    input                mem2reg,   // MemtoReg in Datapath module
    input                alu_src,   // ALUSrc in Datapath module
    input                mem_write, // MemWrite in Datapath module
    input                mem_read,  // MemRead in Datapath module
    input [ALU_CC_W-1:0] alu_cc,    // ALUCC in Datapath module
    output         [6:0] opcode,    // Opcode in Datapath
    output         [6:0] funct7,    // Funct7 in Datapath module
    output         [2:0] funct3,    // Funct3 in Datapath module
    output  wire [DATA_W-1:0] alu_result // ALU_Result in Datapath module
    );
    
    // Initialize wires
    wire             carry_out, zero, overflow;
    wire  [PC_W-1:0] n_pc_count, pc_count;
    wire [IMS_W-1:0] instr_code, wrt_data, alu_in1, alu_in2, rg_data, 
                     immit_value, mem_data;
    
    // The PC counter
    FlipFlop ff1( .clk(clk), 
                  .reset(reset), 
                  .d(n_pc_count), 
                  .q(pc_count));
    
    // Increment the pc counter          
    assign n_pc_count = pc_count + 8'h4;
    
    // The 64x32 Instruction Memory
    InstMem im1( .addr(pc_count),
                 .instruction(instr_code));
    
    // The 32x32 Register File
    RegFile rg1( .clk(clk),
                 .reset(reset),
                 .rg_wrt_en(reg_write),
                 .rg_rd_addr1(instr_code[19:19-RG_ADDRESS+1]),
                 .rg_rd_addr2(instr_code[24:24-RG_ADDRESS+1]),
                 .rg_wrt_addr(instr_code[11:11-RG_ADDRESS+1]),
                 .rg_wrt_data(wrt_data),
                 .rg_rd_data1(alu_in1),
                 .rg_rd_data2(rg_data) );
    
    // A mux to decide if the data from Register File or Immidiate value
    // will be the second input for the ALU             
    MUX21 mux1( .D1(rg_data),
                .D2(immit_value),
                .S(alu_src),
                .Y(alu_in2) );
    
    // Generate the Immidiate value
    ImmGen ig1( .InstCode(instr_code),
                .ImmOut(immit_value) );
       
    // RISC_V processor working on its ALU operation         
    alu_32 a1( .A_in(alu_in1),
               .B_in(alu_in2),
               .ALU_Sel(alu_cc),
               .ALU_Out(alu_result),
               .Carry_Out(carry_out),
               .Zero(zero),
               .Overflow(overflow) );
    
    // The 128x32 Data Memory           
    DataMem dm1( .MemWrite(mem_write),
                 .MemRead(mem_read),
                 .addr(alu_result[DM_ADDRESS-1:0]),
                 .write_data(rg_data),
                 .read_data(mem_data) );
    
    // A 2 to 1 mux that will decide whether the data from memory
    // or ALU will be written to the Register File             
    MUX21 mux2( .D1(alu_result),
                .D2(mem_data),
                .S(mem2reg),
                .Y(wrt_data) );
                
    // Assigning the outputs of the Datapath to part of the instruction code           
    assign opcode = instr_code[6:0];
    assign funct7 = instr_code[31:25];
    assign funct3 = instr_code[14:12];
    
endmodule


module FlipFlop(clk, reset, d, q );

    // input, output, and register initialization
    input   clk, reset;
    input  [7:0] d;
    output [7:0] q;
    
    reg    [7:0] q;
    
    // On the posedge of the clk, if reset q will get 0 for a synchronous reset
    // else q will get the input of d.
    always@(posedge clk) begin
        if(reset)
            q <= 8'b0;
        else
            q <= d;
    end
    
endmodule

module InstMem(addr, instruction );

    // Define input, output, registers, and wires signal
    input  [ 7:0] addr;
    output wire [31:0] instruction;
    
    reg [31:0] memory [63:0];
    
    // The instruction code the RISC-V processor will execute
    initial begin
    memory[0] = 32'h00007033;   // and r0, r0, r0 32'h00000000
    memory[1] = 32'h00100093;   // addi r1, r0, 1 32'h00000001 
    memory[2] = 32'h00200113;   // addi r2, r0, 2 32'h00000002
    memory[3] = 32'h00308193;   // addi r3, r1, 3 32'h00000004 
    memory[4] = 32'h00408213;   // addi r4, r1, 4 32'h00000005
    memory[5] = 32'h00510293;   // addi r5, r2, 5 32'h00000007 
    memory[6] = 32'h00610313;   // addi r6, r2, 6 32'h00000008
    memory[7] = 32'h00718393;   // addi r7, r3, 7 32'h0000000B
    memory[8] = 32'h00208433;   // add r8, r1, r2 32'h00000003 
    memory[9] = 32'h404404B3;   // sub r9, r8, r4 32'hFFFFFFFF or 32'hFFFFFFFE
    memory[10] = 32'h00317533;  // and r10, r2, r3 32'h00000000
    memory[11] = 32'h0041E5B3;  // or r11, r3, r4 32'h00000005 
    memory[12] = 32'h0041A633;  // if r3 is less than r4 then r12 = 1 32'h00000001
    memory[13] = 32'h007346B3;  // nor r13, r6, r7 32'hFFFFFFF4
    memory[14] = 32'h4D34F713;  // andi r14, r9, "4D3" 32'h000004D2
    memory[15] = 32'h8D35E793;  // ori r15, r11, "8D3" 32'hFFFFF8D7
    memory[16] = 32'h4D26A813;  // if r13 is less than 32'h000004D2 then r16 = 1 32'h00000001 
    memory[17] = 32'h4D244893;  // nori r17, r8, "4D2" 32'hFFFFFB2C
    memory[18] = 32'h02B02823;  // sw r11, 48(r0) alu_result = 32'h00000030
    memory[19] = 32'h03002603;  // lw r12, 48(r0) alu_result = 32'h00000030 r12 = 32'h00000005
    end
    
    // Retriving the instruction code based on the PC count
    assign instruction = memory[addr[7:2]];

endmodule



module RegFile(clk, reset, rg_wrt_en, rg_rd_addr1, rg_rd_addr2, 
                rg_wrt_addr, rg_wrt_data, rg_rd_data1, rg_rd_data2 );
                
    // Define input, output, and register
    input         clk, reset, rg_wrt_en;
    input  [ 4:0] rg_rd_addr1, rg_rd_addr2, rg_wrt_addr;
    input  [31:0] rg_wrt_data;
    output [31:0] rg_rd_data1, rg_rd_data2;
    
    reg [31:0] reg16 [31:0];
    
    integer i;
    
    // always block to reset the 32 registers to 0 and if write enable is 
    // active then will write the data into the register.
    always@(posedge clk, posedge reset) begin
        if(reset) begin
            for(i=0; i<32; i= i+1) 
                reg16[i] = 32'b0;
        end
        else
            if(rg_wrt_en)
                reg16[rg_wrt_addr] <= rg_wrt_data;
    end

    // Read the desire register
    assign rg_rd_data1 = reg16[rg_rd_addr1];
    assign rg_rd_data2 = reg16[rg_rd_addr2];
    
endmodule

module MUX21(D1, D2, S, Y );

    // Initialize input and output
    input         S;
    input  [31:0] D1, D2;
    output [31:0] Y;
    
    // If S is active then Y will get D2 else Y will get D1
    assign Y = S ? D2:D1;
    
endmodule
module ImmGen( InstCode, ImmOut  );

    // Initialize input, output, register
    input [31:0] InstCode;
    output [31:0] ImmOut;
    
    reg [31:0] ImmOut;
    
    // always occur on InstCode
    always@(InstCode) begin
        // Case statement that will generate the immidiate value based on the intruction code
        case(InstCode[6:0])
            // Immidiate 
            7'b0000011 : ImmOut = {InstCode[31] ? {20{1'b1}} : 20'b0, InstCode[31:20]};
            // Immidiate addition
            7'b0010011 : ImmOut = {InstCode[31] ? {20{1'b1}} : 20'b0, InstCode[31:20]};
            // Immidiate 
            7'b0100011 : ImmOut = {InstCode[31] ? {20{1'b1}} : 20'b0, InstCode[31:25], InstCode[11:7]};
            // Immidiate
            7'b0010111 : ImmOut = {InstCode[31:12], 12'b0};
            default : ImmOut = 32'b0;
        endcase
    end 
    
endmodule
module alu_32( input  [31:0] A_in, 
			   input  [31:0] B_in, 
			   input  [3:0] ALU_Sel, 
			   output [31:0] ALU_Out, 
			   output reg Carry_Out, 
			   output Zero, 
			   output reg Overflow = 1'b0 );
			   
	reg [31:0] ALU_Result; 
	reg [32:0] temp; 
	reg [32:0] twos_com;
	
	assign ALU_Out = ALU_Result;
	assign Zero    = ( ALU_Result == 0 );
	
	always @(*) begin 
		Overflow  = 1'b0;
		Carry_Out = 1'b0;
		
		case(ALU_Sel)
			4'b0000: //and
				ALU_Result = A_in & B_in;
			
			4'b0001: //or
				ALU_Result = A_in | B_in;
			
			4'b0010: begin 
				ALU_Result = $signed(A_in) + $signed(B_in);
				temp = { 1'b0 , A_in } + { 1'b0 , B_in };
				Carry_Out = temp[32];
				if ( (A_in[31] & B_in[31] & ~ALU_Out[31]) |
					 (~A_in[31] & ~B_in[31] & ALU_Out[31] ))
					 Overflow = 1'b1;
				else 
					Overflow = 1'b0;
			end 
			
			4'b0110: begin //Signed Subtraction with Overflow checking
				ALU_Result = $signed(A_in) - $signed(B_in); 
				twos_com   = ~(B_in) + 1'b1;
				
				if( (A_in[31] & twos_com[31] & ~ALU_Out[31]) |
					(~A_in[31] & ~twos_com[31] & ALU_Out[31]) )
					Overflow = 1'b1;
				else 
					Overflow = 1'b0; 
			
			end
			
			4'b0111: //Signed less than or equal comparison
				ALU_Result = ($signed(A_in) < $signed(B_in))?32'd1:32'd0;
			
			4'b1100: //nor
				ALU_Result = ~(A_in | B_in);
			
			4'b1111: //Comparison
				ALU_Result = ( A_in == B_in )?32'd1:32'd0;
			
			default: ALU_Result = A_in + B_in;
		endcase
	end
endmodule	

module DataMem( MemRead, MemWrite, addr, write_data, read_data );

    // Initiliaze input, output, register, wire, integer
    input         MemRead, MemWrite;
    input  [ 8:0] addr;
    input  [31:0] write_data;
    output [31:0] read_data;
    
    reg  [31:0] memory [127:0];
    wire [31:0] read_data;
    integer i;
    
    // To initialize all 128 registers to zero
    initial begin
        for(i=0; i<128; i=i+1)begin
            memory[i] = 32'b0;
        end
    end
    
    // For any changes to occur if MemWrite is active then the desired memory register 
    // will get the data of write_data
    always@(*) begin            
        if(MemWrite)
            memory[addr] <= write_data;
    end
    
    // If MemRead signal is active then read_data will read the desired memory register
    // else read_data will get a zero
    assign read_data = MemRead ? memory[addr] : 32'b0;
         
endmodule


module Controller( Opcode, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, ALUOp );

    // define the input and output signals
    input [6:0] Opcode;
    output ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite;
    output [1:0] ALUOp;
    
    // define registers
    reg ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite;
    reg [1:0] ALUOp;
    
    // Define the Controller modules behavior
    always@(*)
        case(Opcode)         // Setting all signals based on the OPcode
            7'b0000011 : begin                  // Opcode for Load Word
                         MemtoReg = 1'b1;
                         MemWrite = 1'b0;
                         MemRead = 1'b1;
                         ALUSrc = 1'b1;
                         RegWrite = 1'b1;
                         ALUOp = 2'b01;
                         end
            7'b0100011 : begin                  // Opode for Store Word
                         MemtoReg = 1'b0;
                         MemWrite = 1'b1;
                         MemRead = 1'b0;
                         ALUSrc = 1'b1;
                         RegWrite = 1'b0;
                         ALUOp = 2'b01;
                         end 
            7'b0010011 : begin                  // Opcode for I-type Operation
                         MemtoReg = 1'b0;
                         MemWrite = 1'b0;
                         MemRead = 1'b0;
                         ALUSrc = 1'b1;
                         RegWrite = 1'b1;
                         ALUOp = 2'b00;
                         end
            7'b0110011 :begin                   // Opcode for R-type Operation
                         MemtoReg = 1'b0;
                         MemWrite = 1'b0;
                         MemRead = 1'b0;
                         ALUSrc = 1'b0;
                         RegWrite = 1'b1;
                         ALUOp = 2'b10;
                         end
            default    :begin                   // Set all signals to 0
                         MemtoReg = 1'b0;
                         MemWrite = 1'b0;
                         MemRead = 1'b0;
                         ALUSrc = 1'b0;
                         RegWrite = 1'b0;
                         ALUOp = 2'b00;
                         end
                         
        endcase
    
endmodule


  module ALUController( ALUOp, Funct7, Funct3, Operation );
    
    // Define the input and output signals
    input  [1:0] ALUOp;
    input  [6:0] Funct7;
    input  [2:0] Funct3;
    output [3:0] Operation;  
    reg [3:0] Operation;
    
    // Define the ALUController module behavior
    always@(*) begin
        case({Funct3,ALUOp})
            5'b11110 : Operation = 4'b0000;     // AND operation
            5'b11010 : Operation = 4'b0001;     // OR operation
            5'b10010 : Operation = 4'b1100;     // NOR operation
            5'b01010 : Operation = 4'b0111;     // SLT operation
            5'b00010 : if(Funct7 == 7'h00)
                           Operation = 4'b0010; // ADD operation
                       else if (Funct7 == 7'h20)
                           Operation = 4'b0110; // SUB operation
            5'b11100 : Operation = 4'b0000;     // ANDI operation
            5'b11000 : Operation = 4'b0001;     // ORI operation
            5'b10000 : Operation = 4'b1100;     // NORI operation
            5'b01000 : Operation = 4'b0111;     // SLTI operation
            5'b00000 : Operation = 4'b0010;     // ADDI operation
            5'b01001 : Operation = 4'b0010;     // LW or SW operation
            default  : Operation = 4'b0000;     // default AND operation
        endcase
    end
    
endmodule
