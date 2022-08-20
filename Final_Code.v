`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Texas Tech University
// Engineer: David Gomez
// 
// Create Date:    08/1/2022 
// Module Name:    Final_project
// Project Name:     Project6
// Description:      This is supposed to be the main code used for the CPU. 
//
// Dependencies: N/A
//
// Revision: 3.9
// Additional Comments: Credit given to professor Bilbao since lines of code were 
//		referenced from his lectures. 
//
//////////////////////////////////////////////////////////////////////////////////

module Final_project(clk, aAddress, bAddress, dAddress, aData, bData, dData, registerLoad,
	input1, input2, selection, output_busses, Z, N, C, V,
	rst, ROM, MUXA, MUXB, AddyA, AddyB, RL, IMMEDIATE_VAL, SYMBOLIC_Addy, ALU_, 
	RLA, cs, we, addr, bus, en, addra, douta, 
	clock_100Mhz, reset, Anode_Activate, LED_out);
	 input clock_100Mhz;
    input reset;
    output reg [3:0] Anode_Activate;
    output reg [6:0] LED_out;

	input clk;					 //Clock
	//Register file
	input [3:0] 	aAddress;
	input [3:0] 	bAddress;
	input [3:0] 	dAddress;
	input [31:0] 	dData;
	output [31:0]	aData;
	output [31:0]	bData;
	//ALU
	input 			registerLoad;
	input [15:0]   input1;
	input [15:0]   input2;
	input [5:0]    selection;
	output 		 	Z;
	output  		   N;
	output  		   C;
	output  	 	   V;
	output [31:0]  output_busses;
	//Decoder
	input 			rst;
   input  [31:0]  ROM;
	output [15:0] 	IMMEDIATE_VAL;
	output [15:0] 	SYMBOLIC_Addy;
	output [31:0] 	ALU_;
	output [3:0] 	RLA;
	output [3:0]	AddyA;
	output [3:0]	AddyB;
	output 			MUXA, MUXB, RL;
	//Inputs ROM
	input 			en, cs, we;
	input [3:0] 	addra;
	//Outputs
	output [3:0] 	douta;
	input [3:0] 	addr;
	inout [3:0] 	bus;

	//Necessary arguments
	wire [3:0] din;
	wire [3:0] dout_w;
	reg [3:0] dout;
	
	assign bus = we ? 4'hz :dout;
	assign din = bus;
	
	//ROM Variables
	ROM main (
  .clka(clk), // input clk
  .ena(en), // input en
  .addra(addra), // input addra
  .douta(douta) // output douta
	);
	
	//RAM variables
	RAM ram(
	.clka(clk), 
	.ena(cs), 
	.wea(we), 
	.addra(addr), 
	.dina(din), 
	.douta(dout_w));
	
	 reg [26:0] one_second_counter;
    wire one_second_enable;
    reg [15:0] displayed_number;
    reg [3:0] LED_BCD;
    reg [19:0] refresh_counter;
    wire [1:0] LED_activating_counter;
	 
	//Make sure the changes occur in RAM
	always @(posedge clk) begin
		dout = dout_w;
	end
	
	reg [15:0] 		registers[15:0]; 		//Max register width Max register number
	
	assign aData = registers[aAddress];	//Assign the output data as registers
	assign bData = registers[bAddress];


	reg [15:0] 			results;								
	reg 		  			bit_check1;
	reg		  			bit_check2;
	reg 		  			bit_check3;
	reg 		  			bit_check4;
	 
	assign 				output_busses = results;
	assign 				Z = bit_check1;
	assign 				N = bit_check2;
	assign 				C = bit_check3;
	assign 				V = bit_check4;
	 
	reg 					MUXA;
	reg 					MUXB;
	reg 					RL;
	reg [15:0]   		IMMEDIATE_VAL;
	reg [15:0] 			SYMBOLIC_Addy;
	reg [5:0] 			ALU_;
	reg [3:0] 			AddyA;
	reg [3:0] 			AddyB;
	reg [3:0] 			RLA;
	 	 
	reg [15:0]  		pop;

	reg [2:0] 			REGISTER 	= 3'b000;
	reg [2:0] 			IMMEDIATE = 3'b001;
	reg [2:0] 			SYMBOLIC 	= 3'b001;
	reg [2:0] 			ABSOLUTE 	= 3'b010;
	 
	reg [15:0] 			dst;
	reg [15:0] 			src;
	 
	reg [5:0] 			NOP 	= 6'b000000;
	reg [5:0] 			INC 	= 6'b000001;
	reg [5:0] 			ADDC 	= 6'b000010;
	reg [5:0] 			POP 	= 6'b000011;
	reg [5:0] 			DEC 	= 6'b000100;
	reg [5:0] 			LS  	= 6'b000101;
	reg [5:0] 			RS  	= 6'b000110;
	reg [5:0] 			SET 	= 6'b000111;
	reg [5:0] 			RET 	= 6'b001000;
	reg [5:0] 			JZ  	= 6'b001001;
	reg [5:0] 			JNZ 	= 6'b001010;
	reg [5:0] 			JSR 	= 6'b001011;
	reg [5:0] 			LD_  	= 6'b001100;
	reg [5:0] 			ST  	= 6'b001101;
	reg [5:0] 			ADD  	= 6'b001110;
	reg [5:0] 			SUB 	= 6'b001111;
	reg [5:0] 			AND 	= 6'b010000;
	reg [5:0] 			OR  	= 6'b010001;
	reg [5:0] 			NOT 	= 6'b010010;
	reg [5:0] 			XOR 	= 6'b010011;
	reg [5:0] 			TST 	= 6'b010100;
	reg [5:0] 			PUSH 	= 6'b010101;

	reg 					state;
	reg [5:0] 			Op;
	reg [2:0] 			AS;
	reg [2:0] 			AD;
	 
	always @(negedge clk) begin
		if(registerLoad == 1'b1) begin			//Only load registers when regLoad is High
			registers [dAddress] <= dData;//Register dAddress as dData
		end
	end

    always @(negedge clk) begin
		  bit_check1 = 1'b0;
		  bit_check2 = 1'b0;
		  bit_check3 = 1'b0;
		  bit_check4 = 1'b0;
		  
        if(selection == 6'b000001)begin 		//1
				results = input1 & input2;         //and
        end

        else if(selection == 6'b000010)begin //2
            results = input1 | input2;        //or
        end

        else if(selection == 6'b000011)begin //3
            results = input1 ^ input2;        //xor
        end

        else if(selection == 6'b000100)begin //4
            results = ~input1;        			 //Invert
        end

        else if(selection == 6'b000101)begin //5
            results = input1 + input2;        //add
				if((input1 <= 6'b000000) && (input2 <= 6'b000000)) begin //This is for the V bit
					if(results >= 6'b000000) begin
						bit_check4 = 1'b1;
					end
				end
				if((input1 >= 6'b000000) && (input2 >= 6'b000000)) begin
					if(results <= 6'b000000) begin
						bit_check4 = 1'b1;
					end
				end
		  end

        else if(selection == 6'b000110)begin //6
            results = (input1 + input2) + 1;  //add w carry
        end

        else if(selection == 6'b000111)begin //7
            results = input1 - input2;       //sub
				if((input1 <= 6'b000000) && (input2 >= 6'b000000)) begin //This is for the V bit
					if(results >= 6'b000000) begin
						bit_check4 = 1'b1;
					end
				end
				if((input1 >= 6'b000000) && (input2 <= 6'b000000)) begin
					if(results <= 6'b000000) begin
						bit_check4 = 1'b1;
					end
				end
        end

        else if(selection == 6'b001000)begin //8
            results = input1;                 //Data pass through
        end

        else if(selection == 6'b001001)begin //9
            results = input1<<1;				  	 //Shift left
        end

        else if(selection == 6'b001010)begin //10
            results = input1>>1;				  	 //Shift right
        end
		  
		  if(results == 0) begin 		//Check if value is 0
				bit_check1 = 1'b1;			//This is for the Z bit
		  end
		  if(results < 0) begin			//Check if the value is negative (N)
				bit_check2 = 1'b1;
		  end
		  if(results > 16'hffffffffffffffff) begin			//Does not fit in the width (C)
				bit_check3 = 1'b1;
		  end
		  
    end
	 
    always @(posedge clk) begin
		  
        if(state == 0)begin 				//testing using the and function      
				state<=1;
				Op<=ROM[15:10];
				ALU_<=ROM[15:0];
				AD<=ROM[9:7];
				AS<=ROM[6:4];
				dst<=ROM[3:0];
		  end
		  
		  else begin
		  case(Op)
				ADD: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					AS<=ROM[22:20];
					dst<=ROM[19:16];
					src<=ROM[15:0];
				end
				NOP: 			begin
					Op<=ROM[15:10];
				end
				INC: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AD<=ROM[9:7];
					dst<=ROM[3:0];
				end
				ADDC: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AD<=ROM[9:7];
					dst<=ROM[3:0];
				end
				POP: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AD<=ROM[9:7];
					dst<=ROM[3:0];
				end
				DEC: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AD<=ROM[9:7];
					dst<=ROM[3:0];
				end
				LS: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AD<=ROM[9:7];
					dst<=ROM[3:0];
				end
				RS: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AD<=ROM[9:7];
					dst<=ROM[3:0];
				end
				SET: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AD<=ROM[9:7];
					dst<=ROM[3:0];
				end
				RET: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AS<=ROM[6:4];
					dst<=ROM[3:0];
				end
				JZ: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					dst<=ROM[15:0];
				end
				JNZ: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					dst<=ROM[15:0];
				end
				JSR: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					dst<=ROM[15:0];
				end
				LD_: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					AS<=ROM[22:20];
					dst<=ROM[19:16];
					src<=ROM[15:0];
				end
				ST: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					AS<=ROM[22:20];
					dst<=ROM[19:16];
					src<=ROM[15:0];
				end
				ADD: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					AS<=ROM[22:20];
					dst<=ROM[19:16];
					src<=ROM[15:0];
				end
				SUB: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					AS<=ROM[22:20];
					dst<=ROM[19:16];
					src<=ROM[15:0];
				end
				AND: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					AS<=ROM[22:20];
					dst<=ROM[19:16];
					src<=ROM[15:0];
				end
				OR: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					AS<=ROM[22:20];
					dst<=ROM[19:16];
					src<=ROM[15:0];
				end
				NOT: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AD<=ROM[9:7];
					dst<=ROM[3:0];
				end
				XOR: 			begin
					Op<=ROM[31:26];
					ALU_<=ROM[31:0];
					AD<=ROM[25:23];
					AS<=ROM[22:20];
					dst<=ROM[19:16];
					src<=ROM[15:0];
				end
				TST: 			begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AS<=ROM[6:4];
					dst<=ROM[3:0];
				end
				PUSH: 		begin
					Op<=ROM[15:10];
					ALU_<=ROM[15:0];
					AS<=ROM[6:4];
					dst<=ROM[3:0];
				end
		  endcase
		end
	 end
	
	 task Function();
	  begin
		if(AS == REGISTER) begin
			MUXA<=0;
			AddyA<=ROM[3:0];
		end
		else if(AS == IMMEDIATE) begin
			MUXA<=1;
			IMMEDIATE_VAL<=ROM;
		end

		if(AD == REGISTER) begin
			MUXB<=0;
			AddyB = dst;
			RL<=1;
			RLA<=dst;
		 end
	  end
	 endtask 
    
    always @(posedge clock_100Mhz or posedge reset)
    begin
        if(reset==1)
            one_second_counter <= 0;
        else begin
            if(one_second_counter>=99999999) 
                 one_second_counter <= 0;
            else
                one_second_counter <= one_second_counter + 1;
        end
    end 
    
    assign one_second_enable = (one_second_counter==99999999)?1:0;
    
    always @(posedge clock_100Mhz or posedge reset)
    begin
        if(reset==1)
            displayed_number <= 0;
        else if(one_second_enable==1)
            displayed_number <= displayed_number + 1;
    end
    
    always @(posedge clock_100Mhz or posedge reset)
    begin 
        if(reset==1)
            refresh_counter <= 0;
        else
            refresh_counter <= refresh_counter + 1;
    end
    
    assign LED_activating_counter = refresh_counter[19:18];
    
    always @(*)
    begin
        case(LED_activating_counter)
        2'b00: begin
            Anode_Activate = 4'b0111; 
            LED_BCD = displayed_number[15:11];
             end
        2'b01: begin
            Anode_Activate = 4'b1011; 
            LED_BCD = displayed_number[10:8];
                end
        2'b10: begin
            Anode_Activate = 4'b1101; 
            LED_BCD = displayed_number[7:4];
              end
        2'b11: begin
            Anode_Activate = 4'b1110; 
             LED_BCD = displayed_number[3:0];
               end   
        default:begin
             Anode_Activate = 4'b0111; 
            LED_BCD = displayed_number[15:11];
            end
        endcase
    end
    
    always @(*)
    begin
		  
        case(LED_BCD)
        4'b0000: LED_out = 7'b0000001; // "0"     
        4'b0001: LED_out = 7'b1001111; // "1" 
        4'b0010: LED_out = 7'b0010010; // "2" 
        4'b0011: LED_out = 7'b0000110; // "3" 
        4'b0100: LED_out = 7'b1001100; // "4" 
        4'b0101: LED_out = 7'b0100100; // "5" 
        4'b0110: LED_out = 7'b0100000; // "6" 
        4'b0111: LED_out = 7'b0001111; // "7" 
        4'b1000: LED_out = 7'b0000000; // "8"     
        4'b1001: LED_out = 7'b0000100; // "9" 
        4'b1010: LED_out = 7'b0001000; // a
        4'b1011: LED_out = 7'b1100000; // b
        4'b1100: LED_out = 7'b0110001; // c
        4'b1101: LED_out = 7'b1000010; // d//
        4'b1110: LED_out = 7'b0110000; // e
        4'b1111: LED_out = 7'b0111000; // f//
        default: LED_out = 7'b0000001; // "0"
        endcase
    end
endmodule
