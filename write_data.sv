module write_data
#(
	parameter BUS_NUM 		= 4,
	parameter DOUT_WIDTH 	= 16
)
(
	input 										clk					,
	input 										clk0				,
	input 										rst     	        ,
	input 										clken   	        ,
	input 										din_en_1		    ,      
	input signed [BUS_NUM-1:0][DOUT_WIDTH-1:0]  din_real_1	    	,    
	input signed [BUS_NUM-1:0][DOUT_WIDTH-1:0]  din_imag_1       	,
	
	input 										din_en_2	        ,      
	input signed [BUS_NUM-1:0][DOUT_WIDTH-1:0]  din_real_2	    	,    
	input signed [BUS_NUM-1:0][DOUT_WIDTH-1:0]  din_imag_2       	,
	
	input 										din_en_3	        ,      
	input signed [BUS_NUM-1:0][DOUT_WIDTH-1:0]  din_real_3	    	,    
	input signed [BUS_NUM-1:0][DOUT_WIDTH-1:0]  din_imag_3     	
);

integer fid_0,fid_1,fid_2,fid_3,fid_4,fid_5,fid_6,fid_7; 
integer fid_8,fid_9,fid_10,fid_11,fid_12,fid_13,fid_14,fid_15;
integer fid_16,fid_17,fid_18,fid_19,fid_20,fid_21,fid_22,fid_23;


initial
begin
	fid_0 = $fopen("D:/project_high_speed/matlab/data_save/dout_mod_real_0.txt");
	fid_1 = $fopen("D:/project_high_speed/matlab/data_save/dout_mod_real_1.txt");
	fid_2 = $fopen("D:/project_high_speed/matlab/data_save/dout_mod_real_2.txt");
	fid_3 = $fopen("D:/project_high_speed/matlab/data_save/dout_mod_real_3.txt");
	fid_4 = $fopen("D:/project_high_speed/matlab/data_save/dout_mod_imag_0.txt");
	fid_5 = $fopen("D:/project_high_speed/matlab/data_save/dout_mod_imag_1.txt");
	fid_6 = $fopen("D:/project_high_speed/matlab/data_save/dout_mod_imag_2.txt");
	fid_7 = $fopen("D:/project_high_speed/matlab/data_save/dout_mod_imag_3.txt");
	
end

always@(posedge clk0)
if( din_en_1 == 1)  
begin
	$fwrite(fid_0 ,"%d\n",$signed(din_real_1[0]));
	$fwrite(fid_1 ,"%d\n",$signed(din_real_1[1]));
	$fwrite(fid_2 ,"%d\n",$signed(din_real_1[2]));
	$fwrite(fid_3 ,"%d\n",$signed(din_real_1[3]));       
	$fwrite(fid_4 ,"%d\n",$signed(din_imag_1[0]));     
	$fwrite(fid_5 ,"%d\n",$signed(din_imag_1[1]));     
	$fwrite(fid_6 ,"%d\n",$signed(din_imag_1[2]));     
	$fwrite(fid_7 ,"%d\n",$signed(din_imag_1[3]));          
end 
 
initial
begin
	fid_8 = $fopen("D:/project_high_speed/matlab/data_save/dout_RS_filter_real_0.txt");
	fid_9 = $fopen("D:/project_high_speed/matlab/data_save/dout_RS_filter_real_1.txt");
	fid_10 = $fopen("D:/project_high_speed/matlab/data_save/dout_RS_filter_real_2.txt");
	fid_11 = $fopen("D:/project_high_speed/matlab/data_save/dout_RS_filter_real_3.txt");
	fid_12 = $fopen("D:/project_high_speed/matlab/data_save/dout_RS_filter_imag_0.txt");
	fid_13 = $fopen("D:/project_high_speed/matlab/data_save/dout_RS_filter_imag_1.txt");
	fid_14 = $fopen("D:/project_high_speed/matlab/data_save/dout_RS_filter_imag_2.txt");
	fid_15 = $fopen("D:/project_high_speed/matlab/data_save/dout_RS_filter_imag_3.txt");
	
end

always@(posedge clk)
if( din_en_2 == 1)  
begin
	$fwrite(fid_8  ,"%d\n",$signed(din_real_2[0]));
	$fwrite(fid_9  ,"%d\n",$signed(din_real_2[1]));
	$fwrite(fid_10 ,"%d\n",$signed(din_real_2[2]));
	$fwrite(fid_11 ,"%d\n",$signed(din_real_2[3]));       
	$fwrite(fid_12 ,"%d\n",$signed(din_imag_2[0]));     
	$fwrite(fid_13 ,"%d\n",$signed(din_imag_2[1]));     
	$fwrite(fid_14 ,"%d\n",$signed(din_imag_2[2]));     
	$fwrite(fid_15 ,"%d\n",$signed(din_imag_2[3]));          
end 

initial
begin
	fid_16 = $fopen("D:/project_high_speed/matlab/data_save/dout_zeros_real_0.txt");
	fid_17 = $fopen("D:/project_high_speed/matlab/data_save/dout_zeros_real_1.txt");
	fid_18 = $fopen("D:/project_high_speed/matlab/data_save/dout_zeros_real_2.txt");
	fid_19 = $fopen("D:/project_high_speed/matlab/data_save/dout_zeros_real_3.txt");
	fid_20 = $fopen("D:/project_high_speed/matlab/data_save/dout_zeros_imag_0.txt");
	fid_21 = $fopen("D:/project_high_speed/matlab/data_save/dout_zeros_imag_1.txt");
	fid_22 = $fopen("D:/project_high_speed/matlab/data_save/dout_zeros_imag_2.txt");
	fid_23 = $fopen("D:/project_high_speed/matlab/data_save/dout_zeros_imag_3.txt");
	
end

always@(posedge clk)
if( din_en_3 == 1)  
begin
	$fwrite(fid_16 ,"%d\n",$signed(din_real_3[0]));
	$fwrite(fid_17 ,"%d\n",$signed(din_real_3[1]));
	$fwrite(fid_18 ,"%d\n",$signed(din_real_3[2]));
	$fwrite(fid_19 ,"%d\n",$signed(din_real_3[3]));       
	$fwrite(fid_20 ,"%d\n",$signed(din_imag_3[0]));     
	$fwrite(fid_21 ,"%d\n",$signed(din_imag_3[1]));     
	$fwrite(fid_22 ,"%d\n",$signed(din_imag_3[2]));     
	$fwrite(fid_23 ,"%d\n",$signed(din_imag_3[3]));          
end 











/*

*/

/*
integer fid_0,fid_1,fid_2,fid_3,fid_4,fid_5,fid_6,fid_7; 
integer fid_8,fid_9,fid_10,fid_11,fid_12,fid_13,fid_14,fid_15;
integer fid_16,fid_17,fid_18,fid_19,fid_20,fid_21,fid_22,fid_24; 
initial
begin
	fid_0  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_r_D_r_0.txt");
	fid_1  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_r_D_r_1.txt");
	fid_2  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_r_D_r_2.txt");
	fid_3  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_r_D_r_3.txt");
		   
	fid_4  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_r_D_i_0.txt");
	fid_5  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_r_D_i_1.txt");
	fid_6  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_r_D_i_2.txt");
	fid_7  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_r_D_i_3.txt");
		   
	fid_8  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_i_D_r_0.txt");
	fid_9  = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_i_D_r_1.txt");
	fid_10 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_i_D_r_2.txt");
	fid_11 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_i_D_r_3.txt");
	
	fid_12 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_i_D_i_0.txt");
	fid_13 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_i_D_i_1.txt");
	fid_14 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_i_D_i_2.txt");
	fid_15 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_H_i_D_i_3.txt");
end
    
always@(posedge clk)
if( dout_en1 == 1)  
begin
	$fwrite(fid_0 ,"%d\n",$signed(dout_H_r_D_r[0]));
	$fwrite(fid_1 ,"%d\n",$signed(dout_H_r_D_r[1]));
	$fwrite(fid_2 ,"%d\n",$signed(dout_H_r_D_r[2]));
	$fwrite(fid_3 ,"%d\n",$signed(dout_H_r_D_r[3])); 
				  
	$fwrite(fid_4 ,"%d\n",$signed(dout_H_r_D_i[0]));     
	$fwrite(fid_5 ,"%d\n",$signed(dout_H_r_D_i[1]));     
	$fwrite(fid_6 ,"%d\n",$signed(dout_H_r_D_i[2]));     
	$fwrite(fid_7 ,"%d\n",$signed(dout_H_r_D_i[3]));      

	$fwrite(fid_8 ,"%d\n",$signed(dout_H_i_D_r[0]));
	$fwrite(fid_9 ,"%d\n",$signed(dout_H_i_D_r[1]));
	$fwrite(fid_10,"%d\n",$signed(dout_H_i_D_r[2]));
	$fwrite(fid_11,"%d\n",$signed(dout_H_i_D_r[3]));
	
	$fwrite(fid_12,"%d\n",$signed(dout_H_i_D_i[0]));     
	$fwrite(fid_13,"%d\n",$signed(dout_H_i_D_i[1]));     
	$fwrite(fid_14,"%d\n",$signed(dout_H_i_D_i[2]));     
	$fwrite(fid_15,"%d\n",$signed(dout_H_i_D_i[3]));     
end

initial
begin
	fid_16 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_r_0.txt");
	fid_17 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_r_1.txt");
	fid_18 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_r_2.txt");
	fid_19 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_r_3.txt");
	
	fid_20 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_i_0.txt");
	fid_21 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_i_1.txt");
	fid_22 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_i_2.txt");
	fid_24 = $fopen("F:/realize_PCL/8_XCORR/matlab/dout_i_3.txt");
	
end
*/
endmodule