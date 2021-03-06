library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

library work;
use work.zpu_config.all;
use work.zpuino_config.all;
use work.zpuinopkg.all;
use work.zpupkg.all;
use work.wishbonepkg.all;

entity ishw_slave is
  port (
    wb_clk_i: in std_logic;
	 	wb_rst_i: in std_logic;
    wb_dat_o: out std_logic_vector(wordSize-1 downto 0);
    wb_dat_i: in std_logic_vector(wordSize-1 downto 0);
    wb_adr_i: in std_logic_vector(maxIObit downto minIObit);
    wb_we_i:  in std_logic;
    wb_cyc_i: in std_logic;
    wb_stb_i: in std_logic;
    wb_ack_o: out std_logic;
    wb_inta_o: out std_logic;
    id:       out slot_id;

    -- Wishbone MASTER interface
    mi_wb_dat_i: in std_logic_vector(wordSize-1 downto 0);
    mi_wb_dat_o: out std_logic_vector(wordSize-1 downto 0);
    mi_wb_adr_o: out std_logic_vector(maxAddrBit downto 0);
    mi_wb_sel_o: out std_logic_vector(3 downto 0);
    mi_wb_cti_o: out std_logic_vector(2 downto 0);
    mi_wb_we_o:  out std_logic;
    mi_wb_cyc_o: out std_logic;
    mi_wb_stb_o: out std_logic;
    mi_wb_ack_i: in std_logic;
    mi_wb_stall_i: in std_logic;

    -- signals to data lines
    scki:      in std_logic;
    mosi:      in std_logic;
    miso:      out std_logic
  );
end entity ishw_slave;

architecture behave of ishw_slave is

  component ishw is
  port (
    wb_clk_i: in std_logic;
	 	wb_rst_i: in std_logic;
    wb_dat_o: out std_logic_vector(wordSize-1 downto 0);
    wb_dat_i: in std_logic_vector(wordSize-1 downto 0);
    wb_adr_i: in std_logic_vector(maxIObit downto minIObit);
    wb_we_i:  in std_logic;
    wb_cyc_i: in std_logic;
    wb_stb_i: in std_logic;
    wb_ack_o: out std_logic;
    wb_inta_o: out std_logic;

    -- Wishbone MASTER interface
    mi_wb_dat_i: in std_logic_vector(wordSize-1 downto 0);
    mi_wb_dat_o: out std_logic_vector(wordSize-1 downto 0);
    mi_wb_adr_o: out std_logic_vector(maxAddrBit downto 0);
    mi_wb_sel_o: out std_logic_vector(3 downto 0);
    mi_wb_cti_o: out std_logic_vector(2 downto 0);
    mi_wb_we_o:  out std_logic;
    mi_wb_cyc_o: out std_logic;
    mi_wb_stb_o: out std_logic;
    mi_wb_ack_i: in std_logic;
    mi_wb_stall_i: in std_logic;

    -- signals to data lines
    scki:      in std_logic;
    mosi:      in std_logic;
    miso:      out std_logic
  );
  end component;

begin

  id <= x"05" & x"01";

  ishw_inst: ishw
    port map (
      wb_clk_i      => wb_clk_i,
      wb_rst_i      => wb_rst_i,
      wb_dat_o      => wb_dat_o,
      wb_dat_i      => wb_dat_i,
      wb_adr_i      => wb_adr_i,
      wb_we_i       => wb_we_i,
      wb_cyc_i      => wb_cyc_i,
      wb_stb_i      => wb_stb_i,
      wb_ack_o      => wb_ack_o,
      wb_inta_o     => wb_inta_o,
  
      -- Wishbone MASTER interface
      mi_wb_dat_i   => mi_wb_dat_i,
      mi_wb_dat_o   => mi_wb_dat_o,
      mi_wb_adr_o   => mi_wb_adr_o,
      mi_wb_sel_o   => mi_wb_sel_o,
      mi_wb_cti_o   => mi_wb_cti_o,
      mi_wb_we_o    => mi_wb_we_o,
      mi_wb_cyc_o   => mi_wb_cyc_o,
      mi_wb_stb_o   => mi_wb_stb_o,
      mi_wb_ack_i   => mi_wb_ack_i,
      mi_wb_stall_i => mi_wb_stall_i,
  
      -- signals to data lines
      scki       => scki,
      mosi       => mosi,
      miso       => miso
    );

end architecture;
