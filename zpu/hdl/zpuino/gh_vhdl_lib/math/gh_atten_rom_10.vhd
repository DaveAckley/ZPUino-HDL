---------------------------------------------------------------------
--	Filename:	gh_atten_rom_10.vhd
--			
--	Description:
--		 Attenuation rom, 10 bit address, 16 bit data out (1/8 dB step)
--
--	Copyright (c) 2008 by George Huber 
--		an OpenCores.org Project
--		free to use, but see documentation for conditions 
--
--	Revision 	History:
--	Revision 	Date      	Author   	Comment
--	-------- 	----------	---------	-----------
--	1.0      	11/02/08  	h LeFevre	Initial revision
--	
------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;

entity gh_atten_rom_10 is
	port (
		CLK : in std_logic;
		ADD : in std_logic_vector(9 downto 0);
		Q   : out std_logic_vector(15 downto 0)
		);
end entity;


architecture a of gh_atten_rom_10 is

	signal iADD :  STD_LOGIC_VECTOR(9 DOWNTO 0);
	signal iQ :  STD_LOGIC_VECTOR(15 DOWNTO 0);

	type rom_mem is array (0 to 1023) of std_logic_vector (15 downto 0);
	constant rom : rom_mem :=(  
    x"ffff", x"fc57", x"f8bc", x"f52e", x"f1ad", x"ee39", x"ead2", x"e777", 
    x"e428", x"e0e6", x"ddaf", x"da84", x"d765", x"d451", x"d148", x"ce4b", 
    x"cb58", x"c871", x"c593", x"c2c1", x"bff8", x"bd3a", x"ba86", x"b7dc", 
    x"b53b", x"b2a4", x"b017", x"ad93", x"ab18", x"a8a6", x"a63d", x"a3dd", 
    x"a186", x"9f37", x"9cf1", x"9ab3", x"987d", x"964f", x"9429", x"920b", 
    x"8ff5", x"8de6", x"8bdf", x"89e0", x"87e8", x"85f6", x"840c", x"8229", 
    x"804d", x"7e78", x"7ca9", x"7ae1", x"7920", x"7765", x"75b0", x"7402", 
    x"7259", x"70b7", x"6f1b", x"6d84", x"6bf4", x"6a69", x"68e4", x"6764", 
    x"65ea", x"6475", x"6306", x"619c", x"6037", x"5ed7", x"5d7c", x"5c26", 
    x"5ad5", x"5988", x"5841", x"56fe", x"55c0", x"5486", x"5351", x"5220", 
    x"50f4", x"4fcc", x"4ea8", x"4d88", x"4c6d", x"4b55", x"4a42", x"4932", 
    x"4826", x"471e", x"461a", x"451a", x"441d", x"4324", x"422e", x"413c", 
    x"404e", x"3f62", x"3e7b", x"3d96", x"3cb5", x"3bd7", x"3afc", x"3a24", 
    x"394f", x"387e", x"37af", x"36e4", x"361b", x"3555", x"3492", x"33d1", 
    x"3314", x"3259", x"31a1", x"30eb", x"3038", x"2f88", x"2eda", x"2e2f", 
    x"2d86", x"2cdf", x"2c3b", x"2b9a", x"2afa", x"2a5d", x"29c2", x"2929", 
    x"2893", x"27fe", x"276c", x"26dc", x"264e", x"25c1", x"2537", x"24af", 
    x"2429", x"23a5", x"2322", x"22a2", x"2223", x"21a6", x"212b", x"20b2", 
    x"203a", x"1fc4", x"1f50", x"1ede", x"1e6d", x"1dfe", x"1d90", x"1d24", 
    x"1cb9", x"1c50", x"1be9", x"1b82", x"1b1e", x"1abb", x"1a59", x"19f8", 
    x"199a", x"193c", x"18e0", x"1885", x"182b", x"17d3", x"177b", x"1725", 
    x"16d1", x"167d", x"162b", x"15da", x"158a", x"153b", x"14ee", x"14a1", 
    x"1456", x"140b", x"13c2", x"137a", x"1332", x"12ec", x"12a7", x"1263", 
    x"1220", x"11dd", x"119c", x"115b", x"111c", x"10dd", x"10a0", x"1063", 
    x"1027", x"0fec", x"0fb2", x"0f78", x"0f40", x"0f08", x"0ed1", x"0e9b", 
    x"0e65", x"0e31", x"0dfd", x"0dca", x"0d97", x"0d65", x"0d34", x"0d04", 
    x"0cd5", x"0ca6", x"0c77", x"0c4a", x"0c1d", x"0bf0", x"0bc5", x"0b9a", 
    x"0b6f", x"0b46", x"0b1c", x"0af4", x"0acc", x"0aa4", x"0a7d", x"0a57", 
    x"0a31", x"0a0c", x"09e7", x"09c3", x"099f", x"097c", x"0959", x"0937", 
    x"0915", x"08f4", x"08d3", x"08b3", x"0893", x"0874", x"0855", x"0836", 
    x"0818", x"07fb", x"07de", x"07c1", x"07a4", x"0789", x"076d", x"0752", 
    x"0737", x"071d", x"0703", x"06e9", x"06d0", x"06b7", x"069e", x"0686", 
    x"066e", x"0657", x"063f", x"0629", x"0612", x"05fc", x"05e6", x"05d0", 
    x"05bb", x"05a6", x"0592", x"057d", x"0569", x"0555", x"0542", x"052f", 
    x"051c", x"0509", x"04f6", x"04e4", x"04d2", x"04c1", x"04af", x"049e", 
    x"048d", x"047d", x"046c", x"045c", x"044c", x"043c", x"042d", x"041e", 
    x"040f", x"0400", x"03f1", x"03e3", x"03d5", x"03c7", x"03b9", x"03ab", 
    x"039e", x"0390", x"0383", x"0377", x"036a", x"035d", x"0351", x"0345", 
    x"0339", x"032d", x"0322", x"0316", x"030b", x"0300", x"02f5", x"02ea", 
    x"02df", x"02d5", x"02ca", x"02c0", x"02b6", x"02ac", x"02a2", x"0299", 
    x"028f", x"0286", x"027d", x"0274", x"026b", x"0262", x"0259", x"0251", 
    x"0248", x"0240", x"0238", x"022f", x"0227", x"0220", x"0218", x"0210", 
    x"0209", x"0201", x"01fa", x"01f3", x"01eb", x"01e4", x"01dd", x"01d7", 
    x"01d0", x"01c9", x"01c3", x"01bc", x"01b6", x"01b0", x"01aa", x"01a3", 
    x"019d", x"0198", x"0192", x"018c", x"0186", x"0181", x"017b", x"0176", 
    x"0171", x"016b", x"0166", x"0161", x"015c", x"0157", x"0152", x"014d", 
    x"0148", x"0144", x"013f", x"013b", x"0136", x"0132", x"012d", x"0129", 
    x"0125", x"0121", x"011c", x"0118", x"0114", x"0110", x"010d", x"0109", 
    x"0105", x"0101", x"00fd", x"00fa", x"00f6", x"00f3", x"00ef", x"00ec", 
    x"00e9", x"00e5", x"00e2", x"00df", x"00dc", x"00d8", x"00d5", x"00d2", 
    x"00cf", x"00cc", x"00c9", x"00c6", x"00c4", x"00c1", x"00be", x"00bb", 
    x"00b9", x"00b6", x"00b3", x"00b1", x"00ae", x"00ac", x"00a9", x"00a7", 
    x"00a5", x"00a2", x"00a0", x"009e", x"009b", x"0099", x"0097", x"0095", 
    x"0093", x"0091", x"008f", x"008d", x"008b", x"0089", x"0087", x"0085", 
    x"0083", x"0081", x"007f", x"007d", x"007b", x"007a", x"0078", x"0076", 
    x"0075", x"0073", x"0071", x"0070", x"006e", x"006c", x"006b", x"0069", 
    x"0068", x"0066", x"0065", x"0063", x"0062", x"0061", x"005f", x"005e", 
    x"005d", x"005b", x"005a", x"0059", x"0057", x"0056", x"0055", x"0054", 
    x"0053", x"0051", x"0050", x"004f", x"004e", x"004d", x"004c", x"004b", 
    x"004a", x"0048", x"0047", x"0046", x"0045", x"0044", x"0043", x"0042", 
    x"0042", x"0041", x"0040", x"003f", x"003e", x"003d", x"003c", x"003b", 
    x"003a", x"003a", x"0039", x"0038", x"0037", x"0036", x"0036", x"0035", 
    x"0034", x"0033", x"0033", x"0032", x"0031", x"0030", x"0030", x"002f", 
    x"002e", x"002e", x"002d", x"002c", x"002c", x"002b", x"002b", x"002a", 
    x"0029", x"0029", x"0028", x"0028", x"0027", x"0026", x"0026", x"0025", 
    x"0025", x"0024", x"0024", x"0023", x"0023", x"0022", x"0022", x"0021", 
    x"0021", x"0020", x"0020", x"001f", x"001f", x"001f", x"001e", x"001e", 
    x"001d", x"001d", x"001c", x"001c", x"001c", x"001b", x"001b", x"001a", 
    x"001a", x"001a", x"0019", x"0019", x"0019", x"0018", x"0018", x"0018", 
    x"0017", x"0017", x"0017", x"0016", x"0016", x"0016", x"0015", x"0015", 
    x"0015", x"0014", x"0014", x"0014", x"0014", x"0013", x"0013", x"0013", 
    x"0012", x"0012", x"0012", x"0012", x"0011", x"0011", x"0011", x"0011", 
    x"0010", x"0010", x"0010", x"0010", x"0010", x"000f", x"000f", x"000f", 
    x"000f", x"000e", x"000e", x"000e", x"000e", x"000e", x"000d", x"000d", 
    x"000d", x"000d", x"000d", x"000d", x"000c", x"000c", x"000c", x"000c", 
    x"000c", x"000b", x"000b", x"000b", x"000b", x"000b", x"000b", x"000b", 
    x"000a", x"000a", x"000a", x"000a", x"000a", x"000a", x"000a", x"0009", 
    x"0009", x"0009", x"0009", x"0009", x"0009", x"0009", x"0008", x"0008", 
    x"0008", x"0008", x"0008", x"0008", x"0008", x"0008", x"0008", x"0007", 
    x"0007", x"0007", x"0007", x"0007", x"0007", x"0007", x"0007", x"0007", 
    x"0007", x"0006", x"0006", x"0006", x"0006", x"0006", x"0006", x"0006", 
    x"0006", x"0006", x"0006", x"0006", x"0006", x"0005", x"0005", x"0005", 
    x"0005", x"0005", x"0005", x"0005", x"0005", x"0005", x"0005", x"0005", 
    x"0005", x"0005", x"0005", x"0004", x"0004", x"0004", x"0004", x"0004", 
    x"0004", x"0004", x"0004", x"0004", x"0004", x"0004", x"0004", x"0004", 
    x"0004", x"0004", x"0004", x"0004", x"0003", x"0003", x"0003", x"0003", 
    x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", 
    x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", 
    x"0003", x"0003", x"0003", x"0002", x"0002", x"0002", x"0002", x"0002", 
    x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", 
    x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", 
    x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", 
    x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0001", 
    x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", 
    x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", 
    x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", 
    x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", 
    x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", 
    x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", 
    x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", 
    x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", 
    x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", 
    x"0001", x"0001", x"0001", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", 
    x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000");


begin


PROCESS (CLK)
BEGIN
	if (rising_edge (clk)) then
		Q <= rom(conv_integer(ADD));
	end if;
END PROCESS;


end architecture;
