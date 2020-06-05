// Homework
// Project: MAC
// Function: send packet with crc32 in the end and receive flow 1 and flow data
// Author: Caspar Chen
// E-mail: caspar_chen@pegatroncorp.com
// Date:20200214

`timescale 1ns/1ns
module MAC_Caspar(rst_n, clk, in_sof, in_eof, in_data_v, in_data, o1_etype_b0,
                  o1_etype_b1, o1_full, o2_full, in_full, in_err, o1_sof, o1_eof,
						o1_data_v, o1_data, o2_sof, o2_eof, o2_data_v, o2_data);

	 parameter BUS_WIDTH = 8;	
	 parameter CRC_BITS = 32;
    parameter MAX_MAC_SIZE = 1518;
    parameter MIN_MAC_SIZE = 64;
	 parameter INITIAL_CRC = 32'hffffffff;
	 parameter INITIAL_IN_DATA = 8'h00;
//===========================================================================================	
//crc32 state
//===========================================================================================	
	 parameter [1:0] INITIAL = 2'b00;
	 parameter [1:0] READY = 2'b01;
	 parameter [1:0] CALCULATE = 2'b11;
	 parameter [1:0] DONE = 2'b10;
//===========================================================================================	
//Data input state
//===========================================================================================	
    parameter [1:0] PROCESSING = 2'b01;
	 parameter [1:0] IGNORE = 2'b11;
//===========================================================================================	
//FIFO state
//===========================================================================================	
	 parameter [1:0] BUFFER = 2'b11;
//===========================================================================================	
//switch state
//===========================================================================================
	 parameter [2:0] WAIT = 3'b000;
	 parameter [2:0] POINT = 3'b001;
	 parameter [2:0] JUDGE = 3'b010;  
    parameter [2:0] DROP = 3'b011;
	 parameter [2:0] O1 = 3'b100;
    parameter [2:0] O2 = 3'b101;
    parameter [2:0] FINISH = 3'b110;	 	 
//===========================================================================================	
//Input / Output
//===========================================================================================	
    input rst_n, clk, in_sof, in_eof, in_data_v, o1_full, o2_full;
    input [(BUS_WIDTH-1):0] in_data;
	 input [(BUS_WIDTH-1):0] o1_etype_b0;
    input [(BUS_WIDTH-1):0] o1_etype_b1;
	 
	 output in_full;
	 output reg o1_sof, o1_eof, o1_data_v, o2_sof, o2_eof, o2_data_v;
	 output reg in_err;
	 output reg [(BUS_WIDTH-1):0] o1_data, o2_data;
//reg for State Machine	 
	 reg [1:0] crc32_state_now, crc32_state_next;
	 reg [1:0] input_state_now, input_state_next;
	 reg [1:0] fifo_o1_state_now, fifo_o1_state_next;
	 reg [1:0] fifo_o2_state_now, fifo_o2_state_next;
	 reg [2:0] switch_state_now, switch_state_next;	 
	 reg rd_check;	 
	 reg wr_done_temp_1t, wr_done_temp_2t;	 
	 reg [31:0] package_number;
	 
	 wire [31:0] out_package; 
	 wire [5:0] usedw_package;
	 wire empty_package, full_package;
//===========================================================================================	
//crc32 parameter
//===========================================================================================
    reg crc_check;
//===========================================================================================	
//Delay in_eof as a wr_done for temp(err, package_number, etype)                                
//===========================================================================================  
	 always @ (posedge clk or negedge rst_n) 
	     begin
	         if (!rst_n)
		          begin
					     wr_done_temp_1t <= #1 1'b0;
						  wr_done_temp_2t <= #1 1'b0;
	             end
				else	 
				    begin
					     wr_done_temp_1t <= #1 in_eof;
						  wr_done_temp_2t <= #1 wr_done_temp_1t;		
                end		
		  end
//===========================================================================================	
//Delay 2 cycle for switch_wr_done                             
//===========================================================================================  
	 reg [1:0] switch_wr_done;
	 
    always @ (posedge clk or negedge rst_n) 
	     begin
	         if (!rst_n)
		          begin
					     switch_wr_done <= #1 2'b00;
					 end				
			   else if (switch_state_now == FINISH)
				    begin
					     switch_wr_done <= #1 switch_wr_done + 2'b01;    
	             end		
				else
		          begin
					     switch_wr_done <= #1 2'b0;
					 end
		  end
//===========================================================================================	
//A pulse generation as a rd_check (err, package_number, etype)                                    
//===========================================================================================		  
    reg [1:0] point_counter; 

    always @ (posedge clk or negedge rst_n) 
	     begin
	         if (!rst_n)
		          begin
					     point_counter <= #1 2'b00;
					 end						  
			   else if (switch_state_now == POINT)
				    begin
					     point_counter <= #1 point_counter + 2'b01;    
	             end		
				else
		          begin
					     point_counter <= #1 2'b0;
					 end
		  end
//===========================================================================================
//Judgment mechanism for output O1/O2 channels
//===========================================================================================	
	 wire out_etype; 
	 wire [5:0] usedw_etype;
	 wire empty_etype, full_etype;
	  
    reg [(BUS_WIDTH-1):0] etype_b0_reg, etype_b1_reg;
	 reg compare_etype;
	 
	 wire b1_load, b0_load; 
	 
	 assign b1_load = (package_number == 'd12) ? 1'b1 : 1'b0;	//byte 13 is b1 
	 assign b0_load = (package_number == 'd13) ? 1'b1 : 1'b0;   //byte 14 is b0	 
	 
	always @ (posedge clk or negedge rst_n)
	    begin
	        if (!rst_n)
				   begin
				       etype_b0_reg <= #1 INITIAL_IN_DATA;
				       etype_b1_reg <= #1 INITIAL_IN_DATA;
					end
			  else if (b1_load)
				   begin
				       etype_b1_reg <= #1 in_data;
					end
			  else if (b0_load)
				   begin
		             etype_b0_reg <= #1 in_data;
               end	
			  else
			      begin
					    etype_b1_reg <= #1 etype_b1_reg;
						 etype_b0_reg <= #1 etype_b0_reg;
				   end				
	    end	 
		  
		  	
    always @ (*) begin
	     if ((o1_etype_b0 == etype_b0_reg) && (o1_etype_b1 == etype_b1_reg))
		      begin
				    compare_etype = 1'b1;
				end
		  else if ((o1_etype_b0 != etype_b0_reg) && (o1_etype_b1 != etype_b1_reg))
		      begin
			       compare_etype = 1'b0;
		      end
	     else
	         begin	
			       compare_etype = 1'b0;		      
		      end		
	 end

	FIFO_etype inst_FIFO_etype(
		.clock(clk),
		.data(compare_etype),
		.rdreq(rd_check),
		.sclr(!rst_n),
		.wrreq(wr_done_temp_1t),
		.empty(empty_etype),
		.full(full_etype),
		.q(out_etype),
		.usedw(usedw_etype)
		);		
//===========================================================================================
//Define parameter and module for FIFO_temp	
//===========================================================================================
    reg in_eof_temp, in_sof_temp, in_valid_temp;
	 reg rd_temp, wr_temp;
	 reg [(BUS_WIDTH-1):0] in_data_temp; 
	 wire out_eof_temp, out_sof_temp, out_valid_temp;
	 wire [(BUS_WIDTH-1):0] out_data_temp;
	 wire empty_valid_temp, empty_sof_temp, empty_eof_temp, empty_temp;
	 wire [11:0] usedw_valid_temp, usedw_sof_temp, usedw_eof_temp, usedw_temp;
	 wire full_valid_temp, full_sof_temp, full_eof_temp, full_temp;
	 
	 assign in_full = full_temp;
	
	FIFO_valid inst_FIFO_valid(
		.clock(clk),
		.data(in_valid_temp),
		.rdreq(rd_temp),
		.sclr(!rst_n),
		.wrreq(wr_temp),
		.empty(empty_valid_temp),
		.full(full_valid_temp),
		.q(out_valid_temp),
		.usedw(usedw_valid_temp)
		);
	 
	FIFO_eof inst_FIFO_eof(
		.clock(clk),
		.data(in_eof_temp),
		.rdreq(rd_temp),
		.sclr(!rst_n),
		.wrreq(wr_temp),
		.empty(empty_eof_temp),
		.full(full_eof_temp),
		.q(out_eof_temp),
		.usedw(usedw_eof_temp)
		);
	 
	FIFO_sof inst_FIFO_sof(
		.clock(clk),
		.data(in_sof_temp),
		.rdreq(rd_temp),
		.sclr(!rst_n),
		.wrreq(wr_temp),
		.empty(empty_sof_temp),
		.full(full_sof_temp),
		.q(out_sof_temp),
		.usedw(usedw_sof_temp)
		);
	 
	FIFO_temp inst_FIFO_temp(
		.clock(clk),
		.data(in_data_temp),
		.rdreq(rd_temp),
		.sclr(!rst_n),
		.wrreq(wr_temp),
		.empty(empty_temp),
		.full(full_temp),
		.q(out_data_temp),
		.usedw(usedw_temp)
		);	
//===========================================================================================
//Define parameter and module for FIFO_o1
//===========================================================================================	
    reg in_eof_o1, in_sof_o1, in_valid_o1;
	 reg rd_o1, wr_o1;
	 reg [(BUS_WIDTH-1):0] in_data_o1;
	 wire out_eof_o1, out_sof_o1, out_valid_o1;
	 wire [(BUS_WIDTH-1):0] out_data_o1;
	 wire empty_valid_o1, empty_sof_o1, empty_eof_o1, empty_o1;
	 wire [10:0] usedw_valid_o1, usedw_sof_o1, usedw_eof_o1, usedw_o1;
	 wire full_valid_o1, full_sof_o1, full_eof_o1;
	 wire full_o1;
	
	FIFO_o1eof inst_FIFO_o1eof(
		.clock(clk),
		.data(in_eof_o1),
		.rdreq(rd_o1),
		.sclr(!rst_n),
		.wrreq(wr_o1),
		.empty(empty_eof_o1),
		.full(full_eof_o1),
		.q(out_eof_o1),
		.usedw(usedw_eof_o1)
		);
	 
	FIFO_o1sof inst_FIFO_o1sof(
		.clock(clk),
		.data(in_sof_o1),
		.rdreq(rd_o1),
		.sclr(!rst_n),
		.wrreq(wr_o1),
		.empty(empty_sof_o1),
		.full(full_sof_o1),
		.q(out_sof_o1),
		.usedw(usedw_sof_o1)
		);
		
	FIFO_o1valid inst_FIFO_o1valid(
		.clock(clk),
		.data(in_valid_o1),
		.rdreq(rd_o1),
		.sclr(!rst_n),
		.wrreq(wr_o1),
		.empty(empty_valid_o1),
		.full(full_valid_o1),
		.q(out_valid_o1),
		.usedw(usedw_valid_o1)
		);	
	
  	FIFO_o1 inst_FIFO_o1(
		.clock(clk),
		.data(in_data_o1),
		.rdreq(rd_o1),
		.sclr(!rst_n),
		.wrreq(wr_o1),
		.empty(empty_o1),
		.full(full_o1),
		.q(out_data_o1),
		.usedw(usedw_o1)
		);		
//===========================================================================================
//Define parameter and module for FIFO_o2
//===========================================================================================	
    reg in_eof_o2, in_sof_o2, in_valid_o2;
	 reg rd_o2, wr_o2;
	 reg [(BUS_WIDTH-1):0] in_data_o2;
	 wire out_eof_o2, out_sof_o2, out_valid_o2;
	 wire [(BUS_WIDTH-1):0] out_data_o2;
	 wire empty_valid_o2, empty_sof_o2, empty_eof_o2, empty_o2;
	 wire [10:0] usedw_valid_o2, usedw_sof_o2, usedw_eof_o2, usedw_o2;
	 wire full_valid_o2, full_sof_o2, full_eof_o2;
	 wire full_o2;
	 
	FIFO_o2eof inst_FIFO_o2eof(
		.clock(clk),
		.data(in_eof_o2),
		.rdreq(rd_o2),
		.sclr(!rst_n),
		.wrreq(wr_o2),
		.empty(empty_eof_o2),
		.full(full_eof_o2),
		.q(out_eof_o2),
		.usedw(usedw_eof_o2)
		);
	 
	FIFO_o2sof inst_FIFO_o2sof(
		.clock(clk),
		.data(in_sof_o2),
		.rdreq(rd_o2),
		.sclr(!rst_n),
		.wrreq(wr_o2),
		.empty(empty_sof_o2),
		.full(full_sof_o2),
		.q(out_sof_o2),
		.usedw(usedw_sof_o2)
		);
		
	FIFO_o2valid inst_FIFO_o2valid(
		.clock(clk),
		.data(in_valid_o2),
		.rdreq(rd_o2),
		.sclr(!rst_n),
		.wrreq(wr_o2),
		.empty(empty_valid_o2),
		.full(full_valid_o2),
		.q(out_valid_o2),
		.usedw(usedw_valid_o2)
		);	
		
	FIFO_o2 inst_FIFO_o2(
		.clock(clk),
		.data(in_data_o2),
		.rdreq(rd_o2),
		.sclr(!rst_n),
		.wrreq(wr_o2),
		.empty(empty_o2),
		.full(full_o2),
		.q(out_data_o2),
		.usedw(usedw_o2)
		);	
//===========================================================================================	
//Counter for Package Size                                       
//===========================================================================================	 
	 always @ ( posedge clk or negedge rst_n) 
        begin
	         if (!rst_n)
		          begin
					     package_number <= #1 0;
	             end
		      else if (in_sof && in_data_v)
		          begin
				        package_number <= #1 1;
				    end
            else if (in_data_v) 
		          begin
				        package_number <= #1 (package_number == (MAX_MAC_SIZE + 1)) ? package_number : package_number + 1;  
				    end	
				else if (in_eof && in_data_v)
					 begin
				        package_number <= #1 (package_number == (MAX_MAC_SIZE + 1)) ? package_number : package_number + 1;  
				    end	
		      else begin
                package_number <= #1 package_number;				
		      end		 
        end		  

	FIFO_package inst_FIFO_package(
		.clock(clk),
		.data(package_number),
		.rdreq(rd_check),
		.sclr(!rst_n),
		.wrreq(wr_done_temp_1t),
		.empty(empty_package),
		.full(full_package),
		.q(out_package),
		.usedw(usedw_package)
		);	
//===========================================================================================	 
//State Machine Control for rst_n Timing
//===========================================================================================	  	 
	 always @ (posedge clk or negedge rst_n) begin
	     if (!rst_n)
		      begin
				    crc32_state_now <= #1 INITIAL;					 
					 input_state_now <= #1 INITIAL; 	
					 fifo_o1_state_now <= #1 INITIAL;	
	             fifo_o2_state_now <= #1 INITIAL;
		          switch_state_now <= #1 INITIAL;			 
				end
		  else 
		      begin
				    crc32_state_now <= #1 crc32_state_next;					 
					 input_state_now <= #1 input_state_next; 	
					 fifo_o1_state_now <= #1 fifo_o1_state_next;	
	             fifo_o2_state_now <= #1 fifo_o2_state_next;	
		          switch_state_now <= #1 switch_state_next; 				 
			   end
    end
//===========================================================================================
//State Machine for CRC32                                         
//===========================================================================================	
	 always @ (*) begin
	     case (crc32_state_now)
		      INITIAL:
		          begin
				        crc32_state_next = READY;
						  crc_check = 1'b0;		
					 end			
			   READY:
				    if(in_sof) 
					     begin
								crc32_state_next = CALCULATE;
								crc_check = 1'b0;
					     end 
					 else begin
					     crc32_state_next = READY;
						  crc_check = 1'b0;	
					 end
				CALCULATE:
					 if (in_eof)
					     begin
					         crc32_state_next = DONE;
								crc_check = 1'b0;	
						  end	
				    else begin
						  crc32_state_next = CALCULATE;
						  crc_check = 1'b0;	
					 end
				DONE:
		          if (in_sof)
					     begin
					         crc32_state_next = CALCULATE;
								crc_check = 1'b0;	
				        end
				    else begin
					     crc32_state_next = READY;
						  crc_check = 1'b1;	
					 end
				default:
		          begin

				    end	 
	     endcase    
	 end
//===============================================================================================================//                                        
// CRC32 parallel combinational logic from https://www.easics.com/webtools/crctool                               //                                                                                                             //
// Purpose : synthesizable CRC function                                                                          //
//   * polynomial: x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x^1 + 1 //
//   * data width: 8                                                                                             //
//   convention: the first serial bit is D[7]                                                                    //
//   *procedure:                                                                                                 //
//		1. Initial crc is 0xff ff ff ff                                                                            //
//		2.	Input reflected                                                                                         //
//		3.	Calculating next crc                                                                                    //
//		4. Results reflected                                                                                       //
//		5. Results xor with 0xff ff ff ff                                                                          //                                                                                                     //
//===============================================================================================================//   	  
	 reg [(BUS_WIDTH*4)-1:0] in_data_buffer;
	 reg [(CRC_BITS*4)-1:0] crc_shift_last_4;
	 reg [CRC_BITS-1:0] crc_now;
	 wire [CRC_BITS-1:0] crc_reflected;
	 wire [CRC_BITS-1:0] crc_next;
	 wire [BUS_WIDTH-1:0] in_data_buffer_reflected; 
	 wire compare_crc32;
	 
	 assign compare_crc32 = (crc_shift_last_4[(CRC_BITS*4)-1:(CRC_BITS*3)] == in_data_buffer) ? 1'b1 : 1'b0;	 
//Input reflected	 
	 assign in_data_buffer_reflected[BUS_WIDTH-1] = in_data_buffer[(BUS_WIDTH-8)];
	 assign in_data_buffer_reflected[BUS_WIDTH-2] = in_data_buffer[(BUS_WIDTH-7)];
	 assign in_data_buffer_reflected[BUS_WIDTH-3] = in_data_buffer[(BUS_WIDTH-6)];
	 assign in_data_buffer_reflected[BUS_WIDTH-4] = in_data_buffer[(BUS_WIDTH-5)];
	 assign in_data_buffer_reflected[BUS_WIDTH-5] = in_data_buffer[(BUS_WIDTH-4)];
	 assign in_data_buffer_reflected[BUS_WIDTH-6] = in_data_buffer[(BUS_WIDTH-3)];
	 assign in_data_buffer_reflected[BUS_WIDTH-7] = in_data_buffer[(BUS_WIDTH-2)];
	 assign in_data_buffer_reflected[BUS_WIDTH-8] = in_data_buffer[(BUS_WIDTH-1)];
//Calculating next crc
    assign crc_next[CRC_BITS-32] = in_data_buffer_reflected[6] ^ in_data_buffer_reflected[0] ^ crc_now[24] ^ crc_now[30];
    assign crc_next[CRC_BITS-31] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[6] ^ in_data_buffer_reflected[1] ^ in_data_buffer_reflected[0] ^ crc_now[24] ^ crc_now[25] ^ crc_now[30] ^ crc_now[31];
    assign crc_next[CRC_BITS-30] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[6] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[1] ^ in_data_buffer_reflected[0] ^ crc_now[24] ^ crc_now[25] ^ crc_now[26] ^ crc_now[30] ^ crc_now[31];
    assign crc_next[CRC_BITS-29] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[1] ^ crc_now[25] ^ crc_now[26] ^ crc_now[27] ^ crc_now[31];
    assign crc_next[CRC_BITS-28] = in_data_buffer_reflected[6] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[0] ^ crc_now[24] ^ crc_now[26] ^ crc_now[27] ^ crc_now[28] ^ crc_now[30];
    assign crc_next[CRC_BITS-27] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[6] ^ in_data_buffer_reflected[5] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[1] ^ in_data_buffer_reflected[0] ^ crc_now[24] ^ crc_now[25] ^ crc_now[27] ^ crc_now[28] ^ crc_now[29] ^ crc_now[30] ^ crc_now[31];
    assign crc_next[CRC_BITS-26] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[6] ^ in_data_buffer_reflected[5] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[1] ^ crc_now[25] ^ crc_now[26] ^ crc_now[28] ^ crc_now[29] ^ crc_now[30] ^ crc_now[31];
    assign crc_next[CRC_BITS-25] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[5] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[0] ^ crc_now[24] ^ crc_now[26] ^ crc_now[27] ^ crc_now[29] ^ crc_now[31];
    assign crc_next[CRC_BITS-24] = in_data_buffer_reflected[4] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[1] ^ in_data_buffer_reflected[0] ^ crc_now[0] ^ crc_now[24] ^ crc_now[25] ^ crc_now[27] ^ crc_now[28];
    assign crc_next[CRC_BITS-23] = in_data_buffer_reflected[5] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[1] ^ crc_now[1] ^ crc_now[25] ^ crc_now[26] ^ crc_now[28] ^ crc_now[29];
    assign crc_next[CRC_BITS-22] = in_data_buffer_reflected[5] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[0] ^ crc_now[2] ^ crc_now[24] ^ crc_now[26] ^ crc_now[27] ^ crc_now[29];
    assign crc_next[CRC_BITS-21] = in_data_buffer_reflected[4] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[1] ^ in_data_buffer_reflected[0] ^ crc_now[3] ^ crc_now[24] ^ crc_now[25] ^ crc_now[27] ^ crc_now[28];
    assign crc_next[CRC_BITS-20] = in_data_buffer_reflected[6] ^ in_data_buffer_reflected[5] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[1] ^ in_data_buffer_reflected[0] ^ crc_now[4] ^ crc_now[24] ^ crc_now[25] ^ crc_now[26] ^ crc_now[28] ^ crc_now[29] ^ crc_now[30];
    assign crc_next[CRC_BITS-19] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[6] ^ in_data_buffer_reflected[5] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[1] ^ crc_now[5] ^ crc_now[25] ^ crc_now[26] ^ crc_now[27] ^ crc_now[29] ^ crc_now[30] ^ crc_now[31];
    assign crc_next[CRC_BITS-18] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[6] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[2] ^ crc_now[6] ^ crc_now[26] ^ crc_now[27] ^ crc_now[28] ^ crc_now[30] ^ crc_now[31];
	 assign crc_next[CRC_BITS-17] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[5] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[3] ^ crc_now[7] ^ crc_now[27] ^ crc_now[28] ^ crc_now[29] ^ crc_now[31];
    assign crc_next[CRC_BITS-16] = in_data_buffer_reflected[5] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[0] ^ crc_now[8] ^ crc_now[24] ^ crc_now[28] ^ crc_now[29];
    assign crc_next[CRC_BITS-15] = in_data_buffer_reflected[6] ^ in_data_buffer_reflected[5] ^ in_data_buffer_reflected[1] ^ crc_now[9] ^ crc_now[25] ^ crc_now[29] ^ crc_now[30];
    assign crc_next[CRC_BITS-14] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[6] ^ in_data_buffer_reflected[2] ^ crc_now[10] ^ crc_now[26] ^ crc_now[30] ^ crc_now[31];
    assign crc_next[CRC_BITS-13] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[3] ^ crc_now[11] ^ crc_now[27] ^ crc_now[31];
    assign crc_next[CRC_BITS-12] = in_data_buffer_reflected[4] ^ crc_now[12] ^ crc_now[28];
    assign crc_next[CRC_BITS-11] = in_data_buffer_reflected[5] ^ crc_now[13] ^ crc_now[29];
    assign crc_next[CRC_BITS-10] = in_data_buffer_reflected[0] ^ crc_now[14] ^ crc_now[24];
    assign crc_next[CRC_BITS-9] = in_data_buffer_reflected[6] ^ in_data_buffer_reflected[1] ^ in_data_buffer_reflected[0] ^ crc_now[15] ^ crc_now[24] ^ crc_now[25] ^ crc_now[30];
    assign crc_next[CRC_BITS-8] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[2] ^ in_data_buffer_reflected[1] ^ crc_now[16] ^ crc_now[25] ^ crc_now[26] ^ crc_now[31];
    assign crc_next[CRC_BITS-7] = in_data_buffer_reflected[3] ^ in_data_buffer_reflected[2] ^ crc_now[17] ^ crc_now[26] ^ crc_now[27];
    assign crc_next[CRC_BITS-6] = in_data_buffer_reflected[6] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[3] ^ in_data_buffer_reflected[0] ^ crc_now[18] ^ crc_now[24] ^ crc_now[27] ^ crc_now[28] ^ crc_now[30];
    assign crc_next[CRC_BITS-5] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[5] ^ in_data_buffer_reflected[4] ^ in_data_buffer_reflected[1] ^ crc_now[19] ^ crc_now[25] ^ crc_now[28] ^ crc_now[29] ^ crc_now[31];
    assign crc_next[CRC_BITS-4] = in_data_buffer_reflected[6] ^ in_data_buffer_reflected[5] ^ in_data_buffer_reflected[2] ^ crc_now[20] ^ crc_now[26] ^ crc_now[29] ^ crc_now[30];
    assign crc_next[CRC_BITS-3] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[6] ^ in_data_buffer_reflected[3] ^ crc_now[21] ^ crc_now[27] ^ crc_now[30] ^ crc_now[31];
    assign crc_next[CRC_BITS-2] = in_data_buffer_reflected[7] ^ in_data_buffer_reflected[4] ^ crc_now[22] ^ crc_now[28] ^ crc_now[31];
    assign crc_next[CRC_BITS-1] = in_data_buffer_reflected[5] ^ crc_now[23] ^ crc_now[29];
//crc result refelected and xor with 32'hffffffff
	 assign crc_reflected[CRC_BITS-1] = crc_next[CRC_BITS-32] ^ 1'b1;
  	 assign crc_reflected[CRC_BITS-2] = crc_next[CRC_BITS-31] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-3] = crc_next[CRC_BITS-30] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-4] = crc_next[CRC_BITS-29] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-5] = crc_next[CRC_BITS-28] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-6] = crc_next[CRC_BITS-27] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-7] = crc_next[CRC_BITS-26] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-8] = crc_next[CRC_BITS-25] ^ 1'b1;
    assign crc_reflected[CRC_BITS-9] = crc_next[CRC_BITS-24] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-10] = crc_next[CRC_BITS-23] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-11] = crc_next[CRC_BITS-22] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-12] = crc_next[CRC_BITS-21] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-13] = crc_next[CRC_BITS-20] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-14] = crc_next[CRC_BITS-19] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-15] = crc_next[CRC_BITS-18] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-16] = crc_next[CRC_BITS-17] ^ 1'b1;	
	 assign crc_reflected[CRC_BITS-17] = crc_next[CRC_BITS-16] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-18] = crc_next[CRC_BITS-15] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-19] = crc_next[CRC_BITS-14] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-20] = crc_next[CRC_BITS-13] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-21] = crc_next[CRC_BITS-12] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-22] = crc_next[CRC_BITS-11] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-23] = crc_next[CRC_BITS-10] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-24] = crc_next[CRC_BITS-9] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-25] = crc_next[CRC_BITS-8] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-26] = crc_next[CRC_BITS-7] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-27] = crc_next[CRC_BITS-6] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-28] = crc_next[CRC_BITS-5] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-29] = crc_next[CRC_BITS-4] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-30] = crc_next[CRC_BITS-3] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-31] = crc_next[CRC_BITS-2] ^ 1'b1;
	 assign crc_reflected[CRC_BITS-32] = crc_next[CRC_BITS-1] ^ 1'b1;
	 
	 always @ (posedge clk or negedge rst_n) begin
	     if (!rst_n)
		      begin
				    in_data_buffer[((BUS_WIDTH*4)-1):0] <=#1 32'b0;
				end
		  else if ((in_sof) && (in_data_v))
		      begin
					 in_data_buffer[((BUS_WIDTH*4)-1):0] <=#1 {in_data_buffer[((BUS_WIDTH*3)-1):0], in_data[(BUS_WIDTH-1):0]};
			   end
		  else if ((in_sof == 1'b0) && (in_data_v))
		      begin
					 in_data_buffer[((BUS_WIDTH*4)-1):0] <=#1 {in_data_buffer[((BUS_WIDTH*3)-1):0], in_data[(BUS_WIDTH-1):0]};
		      end
		  else if ((in_eof) && (in_data_v))
		      begin
					 in_data_buffer[((BUS_WIDTH*4)-1):0] <=#1 {in_data_buffer[((BUS_WIDTH*3)-1):0], in_data[(BUS_WIDTH-1):0]};
			   end
		  else begin
		      in_data_buffer <=#1 in_data_buffer;			
		  end
	 end
	 
	 always @ (posedge clk or negedge rst_n) begin
	     if (!rst_n)
		      begin

				end
		  else if ((in_sof) && (in_data_v))
		      begin
                crc_now <= #1 INITIAL_CRC;
			   end
		  else if ((in_sof == 1'b0) && (in_data_v))
		      begin
                crc_now <= #1 crc_next;
		      end
		  else if ((in_eof) && (in_data_v))
		      begin
                crc_now <= #1 crc_next;
			   end
		  else begin
		  
		  end
	 end
	 
	 always @ (posedge clk or negedge rst_n) begin
	     if (!rst_n)
		      begin
					 crc_shift_last_4[(CRC_BITS*4)-1:0] <=#1 128'b0;
				end
		  else if ((in_sof) && (in_data_v))
		      begin
					 crc_shift_last_4[(CRC_BITS*4)-1:0] <=#1 {crc_shift_last_4[95:0], crc_reflected[7:0], crc_reflected[15:8], crc_reflected[23:16], crc_reflected[31:24]};
			   end
		  else if ((in_sof == 1'b0) && (in_data_v))
		      begin
					 crc_shift_last_4[(CRC_BITS*4)-1:0] <=#1 {crc_shift_last_4[95:0], crc_reflected[7:0], crc_reflected[15:8], crc_reflected[23:16], crc_reflected[31:24]};
		      end
		  else if ((in_eof) && (in_data_v))
		      begin
					 crc_shift_last_4[(CRC_BITS*4)-1:0] <=#1 {crc_shift_last_4[95:0], crc_reflected[7:0], crc_reflected[15:8], crc_reflected[23:16], crc_reflected[31:24]};
			   end
		  else begin
            crc_shift_last_4 <=#1 crc_shift_last_4;				
		  end
	 end
//===========================================================================================	
//in_err for CRC32 Check                                      
//===========================================================================================
	 wire out_err; 
	 wire [5:0] usedw_err;
	 wire empty_err, full_err;
	 
	 always @ (posedge clk or negedge rst_n)
	     begin
		      if(!rst_n)
			      begin
			  	       in_err <= #1 1'b0;
			      end
		  else if (crc_check)
		      begin
				    if (compare_crc32)
					     in_err <= #1 1'b0;
					 else
					     in_err <= #1 1'b1;    
			   end			
		  else begin
		      in_err <= #1 1'b0;
		  end
    end 
	 
	 FIFO_err inst_FIFO_err(
		.clock(clk),
		.data(in_err),
		.rdreq(rd_check),
		.sclr(!rst_n),
		.wrreq(wr_done_temp_2t),
		.empty(empty_err),
		.full(full_err),
		.q(out_err),
		.usedw(usedw_err)
		);
//===========================================================================================	
//State Machine for Data Input                                        
//===========================================================================================	
	 always @ (*) begin
        case (input_state_now)
		      INITIAL:
				    begin
					    if (in_sof && in_data_v)
						     begin
						         input_state_next = PROCESSING;
							  end
					    else
						     begin
						         input_state_next = INITIAL;
							  end
				    end
			   PROCESSING: 
				    begin 
					     if (in_eof && in_data_v)
						      begin
						          input_state_next = DONE;
			               end						 
						  else if (!in_data_v)
						      begin
								    input_state_next = IGNORE;								
								end
						  else
						      begin
								    input_state_next = PROCESSING;
								end
				    end
				IGNORE:
				    begin 
					     if (in_data_v)
						      begin
						          input_state_next = PROCESSING;
			               end						 
						  else
						      begin
								    input_state_next = IGNORE;
								end
				    end							
			   DONE:
				    begin
					     if (in_sof && in_data_v)
						      begin
						          input_state_next = PROCESSING;
								end
					     else
						      begin
						          input_state_next = DONE;
								end
				    end
				default:
				    begin
					 
					 end
        endcase
    end
	 
	 always @ (*) begin
        case (input_state_now)
			   INITIAL: 
				    begin
					     wr_temp = 1'b0;  
				    end
			   PROCESSING:
				    begin
					     if (full_temp)
						      begin
					             wr_temp = 1'b0; 
								end
						  else
						      begin 
					             wr_temp = 1'b1;  				 
								end
				    end
				IGNORE:
				    begin
					     wr_temp = 1'b0;  
				    end
			   DONE:
				    begin
					     if (wr_done_temp_1t || wr_done_temp_2t)
						      begin
					             wr_temp = 1'b1;
								end
						  else 
						      begin
						          wr_temp = 1'b0;
						      end	 
				    end
			   default:
			       begin
				 
				    end
        endcase
	 end
	 
	 always @ (posedge clk or negedge rst_n) begin
	     if (!rst_n)
		      begin
			       in_data_temp <= #1 INITIAL_IN_DATA;
					 in_valid_temp <= #1 1'b0;
					 in_eof_temp <= #1 1'b0;
				    in_sof_temp <= #1 1'b0;
		      end
		  else if (full_temp)
		      begin
					 in_valid_temp <= #1 1'b0;
				end
		  else if (wr_done_temp_1t && wr_temp)
		      begin
			       in_data_temp <= #1 INITIAL_IN_DATA;
					 in_valid_temp <= #1 1'b0;
					 in_eof_temp <= #1 1'b0;
				    in_sof_temp <= #1 1'b0;
		      end		  
		  else if (in_data_v && wr_temp)
		      begin
			       in_data_temp <= #1 in_data;
					 in_valid_temp <= #1 in_data_v;
					 in_eof_temp <= #1 in_eof;
				    in_sof_temp <= #1 in_sof;	
				end
		  else if (in_data_v && (!wr_temp))	
		      begin
			       in_data_temp <= #1 in_data;
					 in_valid_temp <= #1 in_data_v;
					 in_eof_temp <= #1 in_eof;
				    in_sof_temp <= #1 in_sof;			  
				end
		  else
		      begin
			       in_data_temp <= #1 INITIAL_IN_DATA;
					 in_valid_temp <= #1 1'b0;
					 in_eof_temp <= #1 1'b0;
				    in_sof_temp <= #1 1'b0;		  
		      end 
	end	
//===========================================================================================	
//State Machine for Switch                                         
//===========================================================================================
	 always @ (*) begin
        case (switch_state_now)
		      WAIT:
				    begin 
                    if (!empty_package || out_eof_temp)
						      begin
						          switch_state_next = POINT;
							   end
						  else
						      begin
								    switch_state_next = WAIT;
								end
			       end
				POINT:
				    begin 
                    if ( point_counter < 2'b01)
						      begin
						          switch_state_next = JUDGE;
							   end
						  else
						      begin
								    switch_state_next = POINT;
								end
			       end
			   JUDGE: 
				    begin 
					     if (out_err || (out_package > MAX_MAC_SIZE) || (out_package < MIN_MAC_SIZE))
						      begin
						          switch_state_next = DROP;
			               end	
				        else if (out_etype)			
						      begin
						          switch_state_next = O1;
							   end
						  else if (!out_etype)
						      begin
						          switch_state_next = O2;								
								end
						  else
						      begin
								    switch_state_next = JUDGE;
								end
				    end
			   DROP: 
				    begin 
					     if (out_eof_temp)
						      begin
									 switch_state_next = FINISH;
			               end						 
						  else
						      begin
						          switch_state_next = DROP;
							   end
				    end
			   O1:
				    begin
					     if (out_eof_temp)
						     begin
									 switch_state_next = FINISH;
								end
					     else
						      begin
						          switch_state_next = O1;
								end
				    end				 
				O2:
				    begin
					     if (out_eof_temp)
						      begin
								    switch_state_next = FINISH;
								end
						  else
								begin
								    switch_state_next = O2;
								end
				    end
			   FINISH:
				    begin
				        if ( switch_wr_done == 2'b10)
						      begin
						          switch_state_next = WAIT;
							   end
						  else
						      begin
								    switch_state_next = FINISH;
								end
					 end	 
				default:
				    begin

					 end
        endcase
    end
	 
	 always @ (*) begin
        case (switch_state_now)
			   WAIT:
				    begin
					     rd_check = 1'b0;
					     rd_temp = 1'b0;
						  wr_o1 = 1'b0;
						  wr_o2 = 1'b0; 
				    end
				POINT:
			       begin
					     rd_check = 1'b1; 
					     rd_temp = 1'b0;
						  wr_o1 = 1'b0;
						  wr_o2 = 1'b0;
				    end	
			   JUDGE:
			       begin
					     rd_check = 1'b0; 
						  wr_o1 = 1'b0;
						  wr_o2 = 1'b0;
						  if (full_o1 || full_o2)
						      begin
								    rd_temp = 1'b0;
								end
						  else
						      begin
					             rd_temp = 1'b1;								
								end
				    end				
			   DROP:
				    begin
					     rd_check = 1'b0;
					     rd_temp = 1'b1;
						  wr_o1 = 1'b0;
						  wr_o2 = 1'b0;
					 end
			   O1:					 
					 begin
					     if (full_o1)
						      begin
								    rd_check = 1'b0;
								    rd_temp = 1'b0;
					             wr_o1 = 1'b0;  
							       wr_o2 = 1'b0;					 
								end
						  else
						      begin
								    rd_check = 1'b0;
								    rd_temp = 1'b1;
					             wr_o1 = 1'b1;	
							       wr_o2 = 1'b0;								 
								end
				    end
			   O2:
				    begin
					     if (full_o2)
						      begin
								    rd_check = 1'b0;
								    rd_temp = 1'b0;
					             wr_o1 = 1'b0;	
					             wr_o2 = 1'b0; 									 
								end
						  else
						      begin
								    rd_check = 1'b0;
								    rd_temp = 1'b1;
									 wr_o1 = 1'b0;	
					             wr_o2 = 1'b1;					 
								end
				    end
		      FINISH:
                begin
					     rd_check = 1'b0;
					     rd_temp = 1'b0;															
						  if (out_etype)
						      begin
								    wr_o1 = 1'b1;
									 wr_o2 = 1'b0;									 
							   end
						  else
						      begin
								    wr_o1 = 1'b0;
                            wr_o2 = 1'b1;								 
							   end	
                end			 
			   default:
			       begin
				 
				    end
        endcase
	 end

	 always @ (posedge clk or negedge rst_n) begin
	     if (!rst_n)
		      begin
					 in_valid_o1 <= #1 1'b0;
					 in_sof_o1 <= #1 1'b0; 
					 in_eof_o1 <= #1 1'b0;
					 in_data_o1 <= #1 INITIAL_IN_DATA;
					 in_valid_o2 <= #1 1'b0;
					 in_sof_o2 <= #1 1'b0; 
					 in_eof_o2 <= #1 1'b0;
					 in_data_o2 <= #1 INITIAL_IN_DATA;			    				 					 
				end			 
		  else if (wr_o1) 
		      begin					 
					 in_valid_o1 <= #1 out_valid_temp;
					 in_sof_o1 <= #1 out_sof_temp; 
					 in_eof_o1 <= #1 out_eof_temp;
					 in_data_o1 <= #1 out_data_temp;
					 in_valid_o2 <= #1 1'b0;
					 in_sof_o2 <= #1 1'b0; 
					 in_eof_o2 <= #1 1'b0;
					 in_data_o2 <= #1 INITIAL_IN_DATA;
				end
		  else if (wr_o2) 
		      begin
				    in_valid_o1 <= #1 1'b0;
					 in_sof_o1 <= #1 1'b0; 
					 in_eof_o1 <= #1 1'b0;
					 in_data_o1 <= #1 INITIAL_IN_DATA;
					 in_valid_o2 <= #1 out_valid_temp;
					 in_sof_o2 <= #1 out_sof_temp; 
					 in_eof_o2 <= #1 out_eof_temp;
					 in_data_o2 <= #1 out_data_temp;
				end
		  else if (full_o1) 
		      begin					 
					 in_valid_o1 <= #1 1'b1;
					 in_valid_o2 <= #1 1'b0;
					 in_sof_o2 <= #1 1'b0; 
					 in_eof_o2 <= #1 1'b0;
					 in_data_o2 <= #1 INITIAL_IN_DATA;
				end
		  else if (full_o2) 
		      begin
				    in_valid_o1 <= #1 1'b0;
					 in_sof_o1 <= #1 1'b0; 
					 in_eof_o1 <= #1 1'b0;
					 in_data_o1 <= #1 INITIAL_IN_DATA;
					 in_valid_o2 <= #1 1'b1;
				end
		  else
		      begin
					 in_valid_o1 <= #1 1'b0;
					 in_sof_o1 <= #1 1'b0; 
					 in_eof_o1 <= #1 1'b0;
					 in_data_o1 <= #1 INITIAL_IN_DATA;
					 in_valid_o2 <= #1 1'b0;
					 in_sof_o2 <= #1 1'b0; 
					 in_eof_o2 <= #1 1'b0;
					 in_data_o2 <= #1 INITIAL_IN_DATA;				
				end
    end
//===========================================================================================	
//State Machine for fifo_o1                                           
//===========================================================================================
		  
	 always @ (*) begin
        case (fifo_o1_state_now)
		      INITIAL:
				    begin
						 if (((!empty_o1) || in_eof_o1) && (!o1_full))
						     begin
						         fifo_o1_state_next = BUFFER;
							  end
					    else
						     begin
						         fifo_o1_state_next = INITIAL;
							  end
				    end
				BUFFER:
		          if (!o1_full) 		
                    begin
			               fifo_o1_state_next = PROCESSING;
                    end	
					 else
                    begin
			               fifo_o1_state_next = BUFFER;
                    end	
			   PROCESSING:
				    begin 
						  if ((o1_eof) && (!o1_full))
						      begin
						          fifo_o1_state_next = DONE;
			               end						 
						  else
						      begin
						          fifo_o1_state_next = PROCESSING;								
								end
				    end
			   DONE:
				    begin
						  if (((!empty_o1) || in_eof_o1) && (!o1_full))
						      begin
						          fifo_o1_state_next = PROCESSING;
								end
					     else
						      begin
						          fifo_o1_state_next = DONE;
								end
				    end
				default:
				    begin
					 
					 end
        endcase
    end
	 
    always @ (*) begin
	     case (fifo_o1_state_now)
			   INITIAL: 
				    begin
				        rd_o1 = 1'b0;
				    end
				BUFFER:
				    begin
					     if (o1_full)
						      begin
			                   rd_o1 = 1'b0;								
								end
						  else
						      begin
			                   rd_o1 = 1'b1;								
								end    					
				    end	
			   PROCESSING:
				    begin
					     if (o1_full)
						      begin
			                   rd_o1 = 1'b0;								
								end
						  else
						      begin
			                   rd_o1 = 1'b1;								
								end    					
				    end
			   DONE:
				    begin
				        rd_o1 = 1'b0;   								
				    end
			   default:
			       begin
				 
				    end
        endcase
	 end
	 
	 always @ (posedge clk or negedge rst_n) begin
	     if (!rst_n)
		      begin
				    o1_data <= #1 INITIAL_IN_DATA;
					 o1_eof <= #1 1'b0;
					 o1_sof <= #1 1'b0;
					 o1_data_v <= #1 1'b0;
		      end
		  else if (rd_o1 && (fifo_o1_state_now == BUFFER))
		      begin
			       o1_data <= #1 INITIAL_IN_DATA;
					 o1_eof <= #1 1'b0;
					 o1_sof <= #1 1'b0;
					 o1_data_v <= #1 1'b0;			
				end
		  else if (rd_o1 && (fifo_o1_state_now == PROCESSING))
		      begin
			       o1_data <= #1 out_data_o1;
					 o1_eof <= #1 out_eof_o1;
					 o1_sof <= #1 out_sof_o1;
					 o1_data_v <= #1 out_valid_o1;				
				end
		 else if (o1_full)
		      begin
					 o1_data_v <= #1 1'b0;
			   end
		 else
		     begin
		  
			  end
	end		
//===========================================================================================	
//State Machine for fifo_o2                                           
//===========================================================================================	 
	 always @ (*) begin
        case (fifo_o2_state_now)
		      INITIAL:
				    begin
						 if (((!empty_o2) || in_eof_o2) && (!o2_full))
						     begin
						         fifo_o2_state_next = BUFFER;
							  end						
					    else
						     begin
							      fifo_o2_state_next = INITIAL;	
							  end
				    end
				BUFFER:
		          if (!o2_full) 		
                    begin
			               fifo_o2_state_next = PROCESSING;
                    end	
					 else
                    begin
			               fifo_o2_state_next = BUFFER;
                    end						     
			   PROCESSING: 
				    begin 
						  if (o2_eof && (!o2_full))
						      begin
						          fifo_o2_state_next = DONE;
			               end						 
						  else
						      begin
						          fifo_o2_state_next = PROCESSING;								
								end
				    end
			   DONE:
				    begin
						  if (((!empty_o2) || in_sof_o2) && (!o2_full))
						      begin
						          fifo_o2_state_next = PROCESSING;
								end
					     else
						      begin
						          fifo_o2_state_next = DONE;							 
								end
				    end
				default:
				    begin
					 
					 end
        endcase
    end
	 
    always @ (*) begin
	     case (fifo_o2_state_now)
			   INITIAL: 
				    begin
				        rd_o2 = 1'b0;   
				    end
				BUFFER:
				    begin
					     if (o2_full)
						      begin
			                   rd_o2 = 1'b0;								
								end
						  else
						      begin
			                   rd_o2 = 1'b1;								
								end    					
				    end				
			   PROCESSING:
				    begin
					     if (o2_full)
						      begin
			                   rd_o2 = 1'b0;								
								end
						  else
						      begin
			                   rd_o2 = 1'b1;								
								end    					
				    end
			   DONE:
				    begin
				        rd_o2 = 1'b0;   								
				    end
			   default:
			       begin
				 
				    end
        endcase
	 end
	 
	 always @ (posedge clk or negedge rst_n) begin
	     if (!rst_n)
		      begin
				    o2_data <= #1 INITIAL_IN_DATA;
					 o2_eof <= #1 1'b0;
					 o2_sof <= #1 1'b0;
					 o2_data_v <= #1 1'b0;
		      end
		  else if (rd_o2 && (fifo_o2_state_now == BUFFER))
		      begin
			       o2_data <= #1 INITIAL_IN_DATA;
					 o2_eof <= #1 1'b0;
					 o2_sof <= #1 1'b0;
					 o2_data_v <= #1 1'b0;			
				end
		  else if (rd_o2 && (fifo_o2_state_now == PROCESSING))
		      begin
			       o2_data <= #1 out_data_o2;
					 o2_eof <= #1 out_eof_o2;
					 o2_sof <= #1 out_sof_o2;
					 o2_data_v <= #1 out_valid_o2;
			   end
		  else if (o2_full)
		      begin
					 o2_data_v <= #1 1'b0;
			   end
		 else
		     begin
		  
			  end
	end	
endmodule