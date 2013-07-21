-- 
-- THIS IS A GENERATED FILE, DO NOT EDIT!  INSTEAD, MAKE CHANGES IN
-- ISHW/components/tiles/zpuino/d8fw/papilio_pro_top.vhd.dat!
--
-- GENERATED: Sun Jul 21 00:35:57 2013 by Ackley 
--
--  ZPUINO implementation on Gadget Factory 'Papilio Pro' Board
-- 
--  Copyright 2011 Alvaro Lopes <alvieboy@alvie.com>
-- 
--  Version: 1.0
-- 
--  The FreeBSD license
--  
--  Redistribution and use in source and binary forms, with or without
--  modification, are permitted provided that the following conditions
--  are met:
--  
--  1. Redistributions of source code must retain the above copyright
--     notice, this list of conditions and the following disclaimer.
--  2. Redistributions in binary form must reproduce the above
--     copyright notice, this list of conditions and the following
--     disclaimer in the documentation and/or other materials
--     provided with the distribution.
--  
--  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY
--  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
--  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
--  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--  ZPU PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
--  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
--  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
--  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
--  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
--  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
--  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--  
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.zpupkg.all;
use work.zpuinopkg.all;
use work.zpuino_config.all;
use work.zpu_config.all;
use work.pad.all;
use work.wishbonepkg.all;

entity papilio_pro_top is
  port (
    CLK:        in std_logic;

    -- Connection to the main SPI flash
    SPI_SCK:    out std_logic;
    SPI_MISO:   in std_logic;
    SPI_MOSI:   out std_logic;
    SPI_CS:     out std_logic;

    -- WING connections
    WING_A:     inout std_logic_vector(15 downto 0);
    WING_B:     inout std_logic_vector(15 downto 0);
    WING_C:     inout std_logic_vector(15 downto 0);

    -- UART (FTDI) connection
    TXD:        out std_logic;
    RXD:        in std_logic;

    DRAM_ADDR   : OUT   STD_LOGIC_VECTOR (12 downto 0);
     DRAM_BA      : OUT   STD_LOGIC_VECTOR (1 downto 0);
     DRAM_CAS_N   : OUT   STD_LOGIC;
     DRAM_CKE      : OUT   STD_LOGIC;
     DRAM_CLK      : OUT   STD_LOGIC;
     DRAM_CS_N   : OUT   STD_LOGIC;
     DRAM_DQ      : INOUT STD_LOGIC_VECTOR(15 downto 0);
     DRAM_DQM      : OUT   STD_LOGIC_VECTOR(1 downto 0);
     DRAM_RAS_N   : OUT   STD_LOGIC;
     DRAM_WE_N    : OUT   STD_LOGIC;

    -- The LED
    LED:        out std_logic
  );
end entity papilio_pro_top;

architecture behave of papilio_pro_top is

  component zpuino_debug_jtag_spartan6 is
  port (
    jtag_data_chain_in: in std_logic_vector(98 downto 0);
    jtag_ctrl_chain_out: out std_logic_vector(11 downto 0)
  );
  end component;

  signal jtag_data_chain_in: std_logic_vector(98 downto 0);
  signal jtag_ctrl_chain_out: std_logic_vector(11 downto 0);

  component clkgen is
  port (
    clkin:  in std_logic;
    rstin:  in std_logic;
    clkout: out std_logic;
    clkout1: out std_logic;
    clkout2: out std_logic;
    clkdiv: out std_logic;
    rstout: out std_logic
  );
  end component;

  component zpuino_serialreset is
  generic (
    SYSTEM_CLOCK_MHZ: integer := 96
  );
  port (
    clk:      in std_logic;
    rx:       in std_logic;
    rstin:    in std_logic;
    rstout:   out std_logic
  );
  end component zpuino_serialreset;

  component wb_bootloader is
  port (
    wb_clk_i:   in std_logic;
    wb_rst_i:   in std_logic;

    wb_dat_o:   out std_logic_vector(31 downto 0);
    wb_adr_i:   in std_logic_vector(11 downto 2);
    wb_cyc_i:   in std_logic;
    wb_stb_i:   in std_logic;
    wb_ack_o:   out std_logic;
    wb_stall_o: out std_logic;

    wb2_dat_o:   out std_logic_vector(31 downto 0);
    wb2_adr_i:   in std_logic_vector(11 downto 2);
    wb2_cyc_i:   in std_logic;
    wb2_stb_i:   in std_logic;
    wb2_ack_o:   out std_logic;
    wb2_stall_o: out std_logic
  );
  end component;

  signal sysrst:      std_logic;
  signal sysclk:      std_logic;
  signal clkgen_rst:  std_logic;
  signal wb_clk_i:    std_logic;
  signal wb_rst_i:    std_logic;

  signal gpio_o:      std_logic_vector(zpuino_gpio_count-1 downto 0);
  signal gpio_t:      std_logic_vector(zpuino_gpio_count-1 downto 0);
  signal gpio_i:      std_logic_vector(zpuino_gpio_count-1 downto 0);

  constant spp_cap_in: std_logic_vector(zpuino_gpio_count-1 downto 0) :=
    "00" &                -- SPI CS and LED
    "1111111111111111" &  -- Wing C
    "0000000000000000" &  -- Wing B
    "1111111111111111";   -- Wing A

  constant spp_cap_out: std_logic_vector(zpuino_gpio_count-1 downto 0) :=
    "00" &                -- SPI CS and LED
    "1111111111111111" &  -- Wing C
    "0000000000000000" &  -- Wing B
    "1111111111111111";   -- Wing A

  -- I/O Signals
  signal slot_cyc:    slot_std_logic_type;
  signal slot_we:     slot_std_logic_type;
  signal slot_stb:    slot_std_logic_type;
  signal slot_read:   slot_cpuword_type;
  signal slot_write:  slot_cpuword_type;
  signal slot_address:slot_address_type;
  signal slot_ack:    slot_std_logic_type;
  signal slot_interrupt: slot_std_logic_type;
  signal slot_id:    slot_id_type;

  -- 2nd SPI signals
  signal spi2_mosi:   std_logic;
  signal spi2_miso:   std_logic;
  signal spi2_sck:    std_logic;

  -- GPIO Periperal Pin Select
  signal gpio_spp_data: std_logic_vector(PPSCOUNT_OUT-1 downto 0);
  signal gpio_spp_read: std_logic_vector(PPSCOUNT_IN-1 downto 0);
  signal ppsout_info_slot: ppsoutinfotype := (others => -1);
  signal ppsout_info_pin:  ppsoutinfotype;
  signal ppsin_info_slot: ppsininfotype := (others => -1);
  signal ppsin_info_pin:  ppsininfotype;

  -- Timer connections
  signal timers_interrupt:  std_logic_vector(1 downto 0);
  signal timers_pwm:        std_logic_vector(1 downto 0);

  -- main SPI signals
  signal spi_pf_miso: std_logic;
  signal spi_pf_mosi: std_logic;
  signal spi_pf_sck:  std_logic;

  -- UART signals
  signal rx: std_logic;
  signal tx: std_logic;
  signal sysclk_sram_we, sysclk_sram_wen: std_ulogic;

  signal ram_wb_ack_o:       std_logic;
  signal ram_wb_dat_i:       std_logic_vector(wordSize-1 downto 0);
  signal ram_wb_dat_o:       std_logic_vector(wordSize-1 downto 0);
  signal ram_wb_adr_i:       std_logic_vector(maxAddrBitIncIO downto 0);
  signal ram_wb_cyc_i:       std_logic;
  signal ram_wb_stb_i:       std_logic;
  signal ram_wb_sel_i:       std_logic_vector(3 downto 0);
  signal ram_wb_we_i:        std_logic;
  signal ram_wb_stall_o:     std_logic;

  signal   m_wb_dat_i:  std_logic_vector(wordSize-1 downto 0);
  signal   m_wb_dat_o:  std_logic_vector(wordSize-1 downto 0);
  signal   m_wb_adr_i:  std_logic_vector(maxAddrBit downto 0);
  signal   m_wb_sel_i:  std_logic_vector(3 downto 0);
  signal   m_wb_cti_i:  std_logic_vector(2 downto 0);
  signal   m_wb_we_i:   std_logic;
  signal   m_wb_cyc_i:  std_logic;
  signal   m_wb_stb_i:  std_logic;
  signal   m_wb_ack_o:  std_logic;
  signal   m_wb_stall_o:  std_logic;

  -- Signal definitions for the NorTh (ishw_nt) face DMA access
  signal   ishw_nt_m_wb_dat_i:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_nt_m_wb_dat_o:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_nt_m_wb_adr_i:  std_logic_vector(maxAddrBit downto 0);
  signal   ishw_nt_m_wb_sel_i:  std_logic_vector(3 downto 0);
  signal   ishw_nt_m_wb_cti_i:  std_logic_vector(2 downto 0);
  signal   ishw_nt_m_wb_we_i:   std_logic;
  signal   ishw_nt_m_wb_cyc_i:  std_logic;
  signal   ishw_nt_m_wb_stb_i:  std_logic;
  signal   ishw_nt_m_wb_ack_o:  std_logic;
  signal   ishw_nt_m_wb_stall_o:  std_logic;

  -- Signal definitions for the NorthEast (ishw_ne) face DMA access
  signal   ishw_ne_m_wb_dat_i:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_ne_m_wb_dat_o:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_ne_m_wb_adr_i:  std_logic_vector(maxAddrBit downto 0);
  signal   ishw_ne_m_wb_sel_i:  std_logic_vector(3 downto 0);
  signal   ishw_ne_m_wb_cti_i:  std_logic_vector(2 downto 0);
  signal   ishw_ne_m_wb_we_i:   std_logic;
  signal   ishw_ne_m_wb_cyc_i:  std_logic;
  signal   ishw_ne_m_wb_stb_i:  std_logic;
  signal   ishw_ne_m_wb_ack_o:  std_logic;
  signal   ishw_ne_m_wb_stall_o:  std_logic;

  -- Signal definitions for the EasT (ishw_et) face DMA access
  signal   ishw_et_m_wb_dat_i:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_et_m_wb_dat_o:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_et_m_wb_adr_i:  std_logic_vector(maxAddrBit downto 0);
  signal   ishw_et_m_wb_sel_i:  std_logic_vector(3 downto 0);
  signal   ishw_et_m_wb_cti_i:  std_logic_vector(2 downto 0);
  signal   ishw_et_m_wb_we_i:   std_logic;
  signal   ishw_et_m_wb_cyc_i:  std_logic;
  signal   ishw_et_m_wb_stb_i:  std_logic;
  signal   ishw_et_m_wb_ack_o:  std_logic;
  signal   ishw_et_m_wb_stall_o:  std_logic;

  -- Signal definitions for the SouthEast (ishw_se) face DMA access
  signal   ishw_se_m_wb_dat_i:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_se_m_wb_dat_o:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_se_m_wb_adr_i:  std_logic_vector(maxAddrBit downto 0);
  signal   ishw_se_m_wb_sel_i:  std_logic_vector(3 downto 0);
  signal   ishw_se_m_wb_cti_i:  std_logic_vector(2 downto 0);
  signal   ishw_se_m_wb_we_i:   std_logic;
  signal   ishw_se_m_wb_cyc_i:  std_logic;
  signal   ishw_se_m_wb_stb_i:  std_logic;
  signal   ishw_se_m_wb_ack_o:  std_logic;
  signal   ishw_se_m_wb_stall_o:  std_logic;

  -- Signal definitions for the SouTh (ishw_st) face DMA access
  signal   ishw_st_m_wb_dat_i:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_st_m_wb_dat_o:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_st_m_wb_adr_i:  std_logic_vector(maxAddrBit downto 0);
  signal   ishw_st_m_wb_sel_i:  std_logic_vector(3 downto 0);
  signal   ishw_st_m_wb_cti_i:  std_logic_vector(2 downto 0);
  signal   ishw_st_m_wb_we_i:   std_logic;
  signal   ishw_st_m_wb_cyc_i:  std_logic;
  signal   ishw_st_m_wb_stb_i:  std_logic;
  signal   ishw_st_m_wb_ack_o:  std_logic;
  signal   ishw_st_m_wb_stall_o:  std_logic;

  -- Signal definitions for the SouthWest (ishw_sw) face DMA access
  signal   ishw_sw_m_wb_dat_i:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_sw_m_wb_dat_o:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_sw_m_wb_adr_i:  std_logic_vector(maxAddrBit downto 0);
  signal   ishw_sw_m_wb_sel_i:  std_logic_vector(3 downto 0);
  signal   ishw_sw_m_wb_cti_i:  std_logic_vector(2 downto 0);
  signal   ishw_sw_m_wb_we_i:   std_logic;
  signal   ishw_sw_m_wb_cyc_i:  std_logic;
  signal   ishw_sw_m_wb_stb_i:  std_logic;
  signal   ishw_sw_m_wb_ack_o:  std_logic;
  signal   ishw_sw_m_wb_stall_o:  std_logic;

  -- Signal definitions for the WesT (ishw_wt) face DMA access
  signal   ishw_wt_m_wb_dat_i:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_wt_m_wb_dat_o:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_wt_m_wb_adr_i:  std_logic_vector(maxAddrBit downto 0);
  signal   ishw_wt_m_wb_sel_i:  std_logic_vector(3 downto 0);
  signal   ishw_wt_m_wb_cti_i:  std_logic_vector(2 downto 0);
  signal   ishw_wt_m_wb_we_i:   std_logic;
  signal   ishw_wt_m_wb_cyc_i:  std_logic;
  signal   ishw_wt_m_wb_stb_i:  std_logic;
  signal   ishw_wt_m_wb_ack_o:  std_logic;
  signal   ishw_wt_m_wb_stall_o:  std_logic;

  -- Signal definitions for the NorthWest (ishw_nw) face DMA access
  signal   ishw_nw_m_wb_dat_i:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_nw_m_wb_dat_o:  std_logic_vector(wordSize-1 downto 0);
  signal   ishw_nw_m_wb_adr_i:  std_logic_vector(maxAddrBit downto 0);
  signal   ishw_nw_m_wb_sel_i:  std_logic_vector(3 downto 0);
  signal   ishw_nw_m_wb_cti_i:  std_logic_vector(2 downto 0);
  signal   ishw_nw_m_wb_we_i:   std_logic;
  signal   ishw_nw_m_wb_cyc_i:  std_logic;
  signal   ishw_nw_m_wb_stb_i:  std_logic;
  signal   ishw_nw_m_wb_ack_o:  std_logic;
  signal   ishw_nw_m_wb_stall_o:  std_logic;

  signal np_ram_wb_ack_o:       std_logic;
  signal np_ram_wb_dat_i:       std_logic_vector(wordSize-1 downto 0);
  signal np_ram_wb_dat_o:       std_logic_vector(wordSize-1 downto 0);
  signal np_ram_wb_adr_i:       std_logic_vector(maxAddrBitIncIO downto 0);
  signal np_ram_wb_cyc_i:       std_logic;
  signal np_ram_wb_stb_i:       std_logic;
  signal np_ram_wb_sel_i:       std_logic_vector(3 downto 0);
  signal np_ram_wb_we_i:        std_logic;

  signal sram_wb_ack_o:       std_logic;
  signal sram_wb_dat_i:       std_logic_vector(wordSize-1 downto 0);
  signal sram_wb_dat_o:       std_logic_vector(wordSize-1 downto 0);
  signal sram_wb_adr_i:       std_logic_vector(maxAddrBitIncIO downto 0);
  signal sram_wb_cyc_i:       std_logic;
  signal sram_wb_stb_i:       std_logic;
  signal sram_wb_we_i:        std_logic;
  signal sram_wb_sel_i:       std_logic_vector(3 downto 0);
  signal sram_wb_stall_o:     std_logic;

  signal rom_wb_ack_o:       std_logic;
  signal rom_wb_dat_o:       std_logic_vector(wordSize-1 downto 0);
  signal rom_wb_adr_i:       std_logic_vector(maxAddrBitIncIO downto 0);
  signal rom_wb_cyc_i:       std_logic;
  signal rom_wb_stb_i:       std_logic;
  signal rom_wb_cti_i:       std_logic_vector(2 downto 0);
  signal rom_wb_stall_o:     std_logic;

  signal sram_rom_wb_ack_o:       std_logic;
  signal sram_rom_wb_dat_o:       std_logic_vector(wordSize-1 downto 0);
  signal sram_rom_wb_adr_i:       std_logic_vector(maxAddrBit downto 2);
  signal sram_rom_wb_cyc_i:       std_logic;
  signal sram_rom_wb_stb_i:       std_logic;
  signal sram_rom_wb_cti_i:       std_logic_vector(2 downto 0);
  signal sram_rom_wb_stall_o:     std_logic;

  signal prom_rom_wb_ack_o:       std_logic;
  signal prom_rom_wb_dat_o:       std_logic_vector(wordSize-1 downto 0);
  signal prom_rom_wb_adr_i:       std_logic_vector(maxAddrBit downto 2);
  signal prom_rom_wb_cyc_i:       std_logic;
  signal prom_rom_wb_stb_i:       std_logic;
  signal prom_rom_wb_cti_i:       std_logic_vector(2 downto 0);
  signal prom_rom_wb_stall_o:     std_logic;

  signal memory_enable: std_logic;

  component sdram_ctrl is
  port (
    wb_clk_i: in std_logic;
	 	wb_rst_i: in std_logic;

    wb_dat_o: out std_logic_vector(31 downto 0);
    wb_dat_i: in std_logic_vector(31 downto 0);
    wb_adr_i: in std_logic_vector(maxIOBit downto minIOBit);
    wb_we_i:  in std_logic;
    wb_cyc_i: in std_logic;
    wb_stb_i: in std_logic;
    wb_sel_i: in std_logic_vector(3 downto 0);
    wb_ack_o: out std_logic;
    wb_stall_o: out std_logic;

    -- extra clocking
    clk_off_3ns: in std_logic;

    -- SDRAM signals
     DRAM_ADDR   : OUT   STD_LOGIC_VECTOR (11 downto 0);
     DRAM_BA      : OUT   STD_LOGIC_VECTOR (1 downto 0);
     DRAM_CAS_N   : OUT   STD_LOGIC;
     DRAM_CKE      : OUT   STD_LOGIC;
     DRAM_CLK      : OUT   STD_LOGIC;
     DRAM_CS_N   : OUT   STD_LOGIC;
     DRAM_DQ      : INOUT STD_LOGIC_VECTOR(15 downto 0);
     DRAM_DQM      : OUT   STD_LOGIC_VECTOR(1 downto 0);
     DRAM_RAS_N   : OUT   STD_LOGIC;
     DRAM_WE_N    : OUT   STD_LOGIC
  
  );
  end component sdram_ctrl;

  component wb_master_np_to_slave_p is
  generic (
    ADDRESS_HIGH: integer := maxIObit;
    ADDRESS_LOW: integer := maxIObit
  );
  port (
    wb_clk_i: in std_logic;
	 	wb_rst_i: in std_logic;

    -- Master signals

    m_wb_dat_o: out std_logic_vector(31 downto 0);
    m_wb_dat_i: in std_logic_vector(31 downto 0);
    m_wb_adr_i: in std_logic_vector(ADDRESS_HIGH downto ADDRESS_LOW);
    m_wb_sel_i: in std_logic_vector(3 downto 0);
    m_wb_cti_i: in std_logic_vector(2 downto 0);
    m_wb_we_i:  in std_logic;
    m_wb_cyc_i: in std_logic;
    m_wb_stb_i: in std_logic;
    m_wb_ack_o: out std_logic;

    -- Slave signals

    s_wb_dat_i: in std_logic_vector(31 downto 0);
    s_wb_dat_o: out std_logic_vector(31 downto 0);
    s_wb_adr_o: out std_logic_vector(ADDRESS_HIGH downto ADDRESS_LOW);
    s_wb_sel_o: out std_logic_vector(3 downto 0);
    s_wb_cti_o: out std_logic_vector(2 downto 0);
    s_wb_we_o:  out std_logic;
    s_wb_cyc_o: out std_logic;
    s_wb_stb_o: out std_logic;
    s_wb_ack_i: in std_logic;
    s_wb_stall_i: in std_logic
  );
  end component;

  -- Signal definitions for the NorTh (NT) face ishw device
  signal ishw_nt_ckom: std_logic; 
  signal ishw_nt_lcki, ishw_nt_lcko, ishw_nt_dati, ishw_nt_dato : std_logic;

  -- Signal definitions for the NorthEast (NE) face ishw device
  signal ishw_ne_ckis: std_logic; 
  signal ishw_ne_lcki, ishw_ne_lcko, ishw_ne_dati, ishw_ne_dato : std_logic;

  -- Signal definitions for the EasT (ET) face ishw device
  signal ishw_et_ckom: std_logic; 
  signal ishw_et_lcki, ishw_et_lcko, ishw_et_dati, ishw_et_dato : std_logic;

  -- Signal definitions for the SouthEast (SE) face ishw device
  signal ishw_se_ckis: std_logic; 
  signal ishw_se_lcki, ishw_se_lcko, ishw_se_dati, ishw_se_dato : std_logic;

  -- Signal definitions for the SouTh (ST) face ishw device
  signal ishw_st_ckis: std_logic; 
  signal ishw_st_lcki, ishw_st_lcko, ishw_st_dati, ishw_st_dato : std_logic;

  -- Signal definitions for the SouthWest (SW) face ishw device
  signal ishw_sw_ckom: std_logic; 
  signal ishw_sw_lcki, ishw_sw_lcko, ishw_sw_dati, ishw_sw_dato : std_logic;

  -- Signal definitions for the WesT (WT) face ishw device
  signal ishw_wt_ckis: std_logic; 
  signal ishw_wt_lcki, ishw_wt_lcko, ishw_wt_dati, ishw_wt_dato : std_logic;

  -- Signal definitions for the NorthWest (NW) face ishw device
  signal ishw_nw_ckom: std_logic; 
  signal ishw_nw_lcki, ishw_nw_lcko, ishw_nw_dati, ishw_nw_dato : std_logic;

  signal clkdiv: std_logic;

begin

  wb_clk_i <= sysclk;
  wb_rst_i <= sysrst;

  rstgen: zpuino_serialreset
    generic map (
      SYSTEM_CLOCK_MHZ  => 96
    )
    port map (
      clk       => sysclk,
      rx        => rx,
      rstin     => clkgen_rst,
      rstout    => sysrst
    );

  clkgen_inst: clkgen
  port map (
    clkin   => clk,
    rstin   => '0'  ,
    clkout  => sysclk,
    clkout1  => sysclk_sram_we,
    clkout2  => sysclk_sram_wen,
    clkdiv   => clkdiv,
    rstout  => clkgen_rst
  );

  -- GPIO (what's left of them) and pad assignments

  ishw_se_lcki <= WING_A(0);   -- pin00: ISHW input; no IPAD sync
  ishw_se_ckis <= WING_A(1);   -- pin01: ISHW input; no IPAD sync
  pin02: OPAD port map(I => ishw_se_lcko,PAD => WING_A(2) );  -- ISHW output
  ishw_se_dati <= WING_A(3);   -- pin03: ISHW input; no IPAD sync
  pin04: OPAD port map(I => ishw_se_dato,PAD => WING_A(4) );  -- ISHW output
  pin05: OPAD port map(I => ishw_et_ckom,PAD => WING_A(5) );  -- ISHW output
  ishw_et_lcki <= WING_A(6);   -- pin06: ISHW input; no IPAD sync
  pin07: OPAD port map(I => ishw_et_lcko,PAD => WING_A(7) );  -- ISHW output
  ishw_et_dati <= WING_A(8);   -- pin08: ISHW input; no IPAD sync
  pin09: OPAD port map(I => ishw_et_dato,PAD => WING_A(9) );  -- ISHW output

  -- pin10 is GPIO
  pin10: IOPAD port map(I => gpio_o(10),O => gpio_i(10),T => gpio_t(10),C => sysclk,PAD => WING_A(10) );

  pin11: OPAD port map(I => ishw_ne_lcko,PAD => WING_A(11) );  -- ISHW output
  ishw_ne_ckis <= WING_A(12);   -- pin12: ISHW input; no IPAD sync
  ishw_ne_lcki <= WING_A(13);   -- pin13: ISHW input; no IPAD sync
  pin14: OPAD port map(I => ishw_ne_dato,PAD => WING_A(14) );  -- ISHW output
  ishw_ne_dati <= WING_A(15);   -- pin15: ISHW input; no IPAD sync
  pin16: OPAD port map(I => ishw_nt_ckom,PAD => WING_B(0) );  -- ISHW output
  ishw_nt_lcki <= WING_B(1);   -- pin17: ISHW input; no IPAD sync
  pin18: OPAD port map(I => ishw_nt_lcko,PAD => WING_B(2) );  -- ISHW output
  ishw_nt_dati <= WING_B(3);   -- pin19: ISHW input; no IPAD sync
  pin20: OPAD port map(I => ishw_nt_dato,PAD => WING_B(4) );  -- ISHW output

  -- pin21 is GPIO
  pin21: IOPAD port map(I => gpio_o(21),O => gpio_i(21),T => gpio_t(21),C => sysclk,PAD => WING_B(5) );


  -- pin22 is GPIO
  pin22: IOPAD port map(I => gpio_o(22),O => gpio_i(22),T => gpio_t(22),C => sysclk,PAD => WING_B(6) );


  -- pin23 is GPIO
  pin23: IOPAD port map(I => gpio_o(23),O => gpio_i(23),T => gpio_t(23),C => sysclk,PAD => WING_B(7) );


  -- pin24 is GPIO
  pin24: IOPAD port map(I => gpio_o(24),O => gpio_i(24),T => gpio_t(24),C => sysclk,PAD => WING_B(8) );


  -- pin25 is GPIO
  pin25: IOPAD port map(I => gpio_o(25),O => gpio_i(25),T => gpio_t(25),C => sysclk,PAD => WING_B(9) );

  ishw_st_dati <= WING_B(10);   -- pin26: ISHW input; no IPAD sync
  pin27: OPAD port map(I => ishw_st_dato,PAD => WING_B(11) );  -- ISHW output
  ishw_st_lcki <= WING_B(12);   -- pin28: ISHW input; no IPAD sync
  pin29: OPAD port map(I => ishw_st_lcko,PAD => WING_B(13) );  -- ISHW output
  ishw_st_ckis <= WING_B(14);   -- pin30: ISHW input; no IPAD sync

  -- pin31 is GPIO
  pin31: IOPAD port map(I => gpio_o(31),O => gpio_i(31),T => gpio_t(31),C => sysclk,PAD => WING_B(15) );

  pin32: OPAD port map(I => ishw_nw_lcko,PAD => WING_C(0) );  -- ISHW output
  pin33: OPAD port map(I => ishw_nw_ckom,PAD => WING_C(1) );  -- ISHW output
  ishw_nw_lcki <= WING_C(2);   -- pin34: ISHW input; no IPAD sync
  pin35: OPAD port map(I => ishw_nw_dato,PAD => WING_C(3) );  -- ISHW output
  ishw_nw_dati <= WING_C(4);   -- pin36: ISHW input; no IPAD sync

  -- pin37 is GPIO
  pin37: IOPAD port map(I => gpio_o(37),O => gpio_i(37),T => gpio_t(37),C => sysclk,PAD => WING_C(5) );

  ishw_wt_dati <= WING_C(6);   -- pin38: ISHW input; no IPAD sync
  pin39: OPAD port map(I => ishw_wt_dato,PAD => WING_C(7) );  -- ISHW output
  ishw_wt_lcki <= WING_C(8);   -- pin40: ISHW input; no IPAD sync
  pin41: OPAD port map(I => ishw_wt_lcko,PAD => WING_C(9) );  -- ISHW output
  ishw_wt_ckis <= WING_C(10);   -- pin42: ISHW input; no IPAD sync
  ishw_sw_lcki <= WING_C(11);   -- pin43: ISHW input; no IPAD sync
  pin44: OPAD port map(I => ishw_sw_ckom,PAD => WING_C(12) );  -- ISHW output
  pin45: OPAD port map(I => ishw_sw_lcko,PAD => WING_C(13) );  -- ISHW output
  ishw_sw_dati <= WING_C(14);   -- pin46: ISHW input; no IPAD sync
  pin47: OPAD port map(I => ishw_sw_dato,PAD => WING_C(15) );  -- ISHW output

  -- Other ports are special, we need to avoid outputs on input-only pins

  ibufrx:   IPAD port map ( PAD => RXD,        O => rx,           C => sysclk );
  ibufmiso: IPAD port map ( PAD => SPI_MISO,   O => spi_pf_miso,  C => sysclk );

  obuftx:   OPAD port map ( I => tx,           PAD => TXD );
  ospiclk:  OPAD port map ( I => spi_pf_sck,   PAD => SPI_SCK );
  ospics:   OPAD port map ( I => gpio_o(48),   PAD => SPI_CS );
  ospimosi: OPAD port map ( I => spi_pf_mosi,  PAD => SPI_MOSI );
  oled:     OPAD port map ( I => gpio_o(49),   PAD => LED );

  zpuino:zpuino_top_icache
    port map (
      clk           => sysclk,
	 	  rst           => sysrst,

      slot_cyc      => slot_cyc,
      slot_we       => slot_we,
      slot_stb      => slot_stb,
      slot_read     => slot_read,
      slot_write    => slot_write,
      slot_address  => slot_address,
      slot_ack      => slot_ack,
      slot_interrupt=> slot_interrupt,
      slot_id       => slot_id,

      pps_in_slot   => ppsin_info_slot,
      pps_in_pin    => ppsin_info_pin,

      pps_out_slot => ppsout_info_slot,
      pps_out_pin  => ppsout_info_pin,

      m_wb_dat_o    => m_wb_dat_o,
      m_wb_dat_i    => m_wb_dat_i,
      m_wb_adr_i(maxAddrBitIncIO downto maxAddrBit+1) => "XXXXX",
      m_wb_adr_i(maxAddrBit downto 0)    => m_wb_adr_i,
      m_wb_we_i     => m_wb_we_i,
      m_wb_cyc_i    => m_wb_cyc_i,
      m_wb_stb_i    => m_wb_stb_i,
      m_wb_ack_o    => m_wb_ack_o,
      m_wb_stall_o  => m_wb_stall_o,

      wb_ack_i      => sram_wb_ack_o,
      wb_stall_i    => sram_wb_stall_o,
      wb_dat_o      => sram_wb_dat_i,
      wb_dat_i      => sram_wb_dat_o,
      wb_adr_o      => sram_wb_adr_i(maxAddrBit downto 0),
      wb_cyc_o      => sram_wb_cyc_i,
      wb_stb_o      => sram_wb_stb_i,
      wb_sel_o      => sram_wb_sel_i,
      wb_we_o       => sram_wb_we_i,

      -- No debug unit connected
      dbg_reset     => open,
      jtag_data_chain_out => open,            --jtag_data_chain_in,
      jtag_ctrl_chain_in  => (others => '0') --jtag_ctrl_chain_out
    );

  --
  -- IO SLOT 1
  --

  uart_inst: zpuino_uart
  port map (
    wb_clk_i      => wb_clk_i,
	 	wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(1),
    wb_dat_i      => slot_write(1),
    wb_adr_i      => slot_address(1),
    wb_we_i       => slot_we(1),
    wb_cyc_i      => slot_cyc(1),
    wb_stb_i      => slot_stb(1),
    wb_ack_o      => slot_ack(1),
    wb_inta_o     => slot_interrupt(1),
    id            => slot_id(1),

    enabled       => open,
    tx            => tx,
    rx            => rx
  );

  --
  -- IO SLOT 2
  --

  gpio_inst: zpuino_gpio
  generic map (
    gpio_count => zpuino_gpio_count
  )
  port map (
    wb_clk_i      => wb_clk_i,
	 	wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(2),
    wb_dat_i      => slot_write(2),
    wb_adr_i      => slot_address(2),
    wb_we_i       => slot_we(2),
    wb_cyc_i      => slot_cyc(2),
    wb_stb_i      => slot_stb(2),
    wb_ack_o      => slot_ack(2),
    wb_inta_o     => slot_interrupt(2),
    id            => slot_id(2),

    spp_data      => gpio_spp_data,
    spp_read      => gpio_spp_read,

    gpio_i        => gpio_i,
    gpio_t        => gpio_t,
    gpio_o        => gpio_o,
    spp_cap_in    => spp_cap_in,
    spp_cap_out   => spp_cap_out
  );

  --
  -- IO SLOT 3
  --

  timers_inst: zpuino_timers
  generic map (
    A_TSCENABLED        => true,
    A_PWMCOUNT          => 1,
    A_WIDTH             => 32,
    A_PRESCALER_ENABLED => true,
    A_BUFFERS           => true,
    B_TSCENABLED        => false,
    B_PWMCOUNT          => 1,
    B_WIDTH             => 24,
    B_PRESCALER_ENABLED => true,
    B_BUFFERS           => false
  )
  port map (
    wb_clk_i      => wb_clk_i,
	 	wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(3),
    wb_dat_i      => slot_write(3),
    wb_adr_i      => slot_address(3),
    wb_we_i       => slot_we(3),
    wb_cyc_i      => slot_cyc(3),
    wb_stb_i      => slot_stb(3),
    wb_ack_o      => slot_ack(3),
    id            => slot_id(3),

    wb_inta_o     => slot_interrupt(3), -- We use two interrupt lines
    wb_intb_o     => slot_interrupt(4), -- so we borrow intr line from slot 4

    pwm_a_out   => timers_pwm(0 downto 0),
    pwm_b_out   => timers_pwm(1 downto 1)
  );

  --
  -- IO SLOT 4
  --

  slot4: zpuino_spi
  port map (
    wb_clk_i      => wb_clk_i,
	 	wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(4),
    wb_dat_i      => slot_write(4),
    wb_adr_i      => slot_address(4),
    wb_we_i       => slot_we(4),
    wb_cyc_i      => slot_cyc(4),
    wb_stb_i      => slot_stb(4),
    wb_ack_o      => slot_ack(4),
    -- wb_inta_o     => slot_interrupt(4), -- Used by the Timers.
    id            => slot_id(4),

    mosi          => spi_pf_mosi,
    miso          => spi_pf_miso,
    sck           => spi_pf_sck,
    enabled       => open
  );

  --
  -- IO SLOT 5
  --

  slot5: ishw_master
  port map (
    Wb_clk_i      => wb_clk_i,
    wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(5),
    wb_dat_i      => slot_write(5),
    wb_adr_i      => slot_address(5),
    wb_we_i       => slot_we(5),
    wb_cyc_i      => slot_cyc(5),
    wb_stb_i      => slot_stb(5),
    wb_ack_o      => slot_ack(5),
    wb_inta_o     => slot_interrupt(5),
    id            => slot_id(5),

    mi_wb_dat_i   => ishw_nt_m_wb_dat_o,
    mi_wb_dat_o   => ishw_nt_m_wb_dat_i,
    mi_wb_adr_o   => ishw_nt_m_wb_adr_i(maxAddrBit downto 0),
    mi_wb_sel_o   => ishw_nt_m_wb_sel_i,
    mi_wb_cti_o   => ishw_nt_m_wb_cti_i,
    mi_wb_we_o    => ishw_nt_m_wb_we_i,
    mi_wb_cyc_o   => ishw_nt_m_wb_cyc_i,
    mi_wb_stb_o   => ishw_nt_m_wb_stb_i,
    mi_wb_ack_i   => ishw_nt_m_wb_ack_o,
    mi_wb_stall_i => ishw_nt_m_wb_stall_o,

    txclk      => clkdiv,
    scko       => ishw_nt_ckom,
    mosi       => ishw_nt_dato,
    miso       => ishw_nt_dati
  );

  --
  -- IO SLOT 6
  --

  slot6: zpuino_spi
  port map (
    wb_clk_i      => wb_clk_i,
	 	wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(6),
    wb_dat_i      => slot_write(6),
    wb_adr_i      => slot_address(6),
    wb_we_i       => slot_we(6),
    wb_cyc_i      => slot_cyc(6),
    wb_stb_i      => slot_stb(6),
    wb_ack_o      => slot_ack(6),
    wb_inta_o     => slot_interrupt(6),
    id            => slot_id(6),

    mosi          => spi2_mosi,
    miso          => spi2_miso,
    sck           => spi2_sck,
    enabled       => open
  );



  --
  -- IO SLOT 7
  --

  crc16_inst: zpuino_crc16
  port map (
    wb_clk_i      => wb_clk_i,
	 	wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(7),
    wb_dat_i      => slot_write(7),
    wb_adr_i      => slot_address(7),
    wb_we_i       => slot_we(7),
    wb_cyc_i      => slot_cyc(7),
    wb_stb_i      => slot_stb(7),
    wb_ack_o      => slot_ack(7),
    wb_inta_o     => slot_interrupt(7),
    id            => slot_id(7)
  );

  --
  -- IO SLOT 8
  --

  slot8: ishw_slave
  port map (
    Wb_clk_i      => wb_clk_i,
    wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(8),
    wb_dat_i      => slot_write(8),
    wb_adr_i      => slot_address(8),
    wb_we_i       => slot_we(8),
    wb_cyc_i      => slot_cyc(8),
    wb_stb_i      => slot_stb(8),
    wb_ack_o      => slot_ack(8),
    wb_inta_o     => slot_interrupt(8),
    id            => slot_id(8),

    mi_wb_dat_i   => ishw_ne_m_wb_dat_o,
    mi_wb_dat_o   => ishw_ne_m_wb_dat_i,
    mi_wb_adr_o   => ishw_ne_m_wb_adr_i(maxAddrBit downto 0),
    mi_wb_sel_o   => ishw_ne_m_wb_sel_i,
    mi_wb_cti_o   => ishw_ne_m_wb_cti_i,
    mi_wb_we_o    => ishw_ne_m_wb_we_i,
    mi_wb_cyc_o   => ishw_ne_m_wb_cyc_i,
    mi_wb_stb_o   => ishw_ne_m_wb_stb_i,
    mi_wb_ack_i   => ishw_ne_m_wb_ack_o,
    mi_wb_stall_i => ishw_ne_m_wb_stall_o,

    scki       => ishw_ne_ckis,
    mosi       => ishw_ne_dati,
    miso       => ishw_ne_dato
  );

  sram_inst: sdram_ctrl
    port map (
      wb_clk_i    => wb_clk_i,
  	 	wb_rst_i    => wb_rst_i,
      wb_dat_o    => sram_wb_dat_o,
      wb_dat_i    => sram_wb_dat_i,
      wb_adr_i    => sram_wb_adr_i(maxIObit downto minIObit),
      wb_we_i     => sram_wb_we_i,
      wb_cyc_i    => sram_wb_cyc_i,
      wb_stb_i    => sram_wb_stb_i,
      wb_sel_i    => sram_wb_sel_i,
      --wb_cti_i    => CTI_CYCLE_CLASSIC,
      wb_ack_o    => sram_wb_ack_o,
      wb_stall_o  => sram_wb_stall_o,

      clk_off_3ns => sysclk_sram_we,
    DRAM_ADDR   => DRAM_ADDR(11 downto 0),
    DRAM_BA     => DRAM_BA,
    DRAM_CAS_N  => DRAM_CAS_N,
    DRAM_CKE    => DRAM_CKE,
    DRAM_CLK    => DRAM_CLK,
    DRAM_CS_N   => DRAM_CS_N,
    DRAM_DQ     => DRAM_DQ,
    DRAM_DQM    => DRAM_DQM,
    DRAM_RAS_N  => DRAM_RAS_N,
    DRAM_WE_N   => DRAM_WE_N

    );
    DRAM_ADDR(12) <= '0';

  --
  -- IO SLOT 9
  --

  slot9: ishw_master
  port map (
    Wb_clk_i      => wb_clk_i,
    wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(9),
    wb_dat_i      => slot_write(9),
    wb_adr_i      => slot_address(9),
    wb_we_i       => slot_we(9),
    wb_cyc_i      => slot_cyc(9),
    wb_stb_i      => slot_stb(9),
    wb_ack_o      => slot_ack(9),
    wb_inta_o     => slot_interrupt(9),
    id            => slot_id(9),

    mi_wb_dat_i   => ishw_et_m_wb_dat_o,
    mi_wb_dat_o   => ishw_et_m_wb_dat_i,
    mi_wb_adr_o   => ishw_et_m_wb_adr_i(maxAddrBit downto 0),
    mi_wb_sel_o   => ishw_et_m_wb_sel_i,
    mi_wb_cti_o   => ishw_et_m_wb_cti_i,
    mi_wb_we_o    => ishw_et_m_wb_we_i,
    mi_wb_cyc_o   => ishw_et_m_wb_cyc_i,
    mi_wb_stb_o   => ishw_et_m_wb_stb_i,
    mi_wb_ack_i   => ishw_et_m_wb_ack_o,
    mi_wb_stall_i => ishw_et_m_wb_stall_o,

    txclk      => clkdiv,
    scko       => ishw_et_ckom,
    mosi       => ishw_et_dato,
    miso       => ishw_et_dati
  );

  --
  -- IO SLOT 10
  --

  slot10: ishw_slave
  port map (
    Wb_clk_i      => wb_clk_i,
    wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(10),
    wb_dat_i      => slot_write(10),
    wb_adr_i      => slot_address(10),
    wb_we_i       => slot_we(10),
    wb_cyc_i      => slot_cyc(10),
    wb_stb_i      => slot_stb(10),
    wb_ack_o      => slot_ack(10),
    wb_inta_o     => slot_interrupt(10),
    id            => slot_id(10),

    mi_wb_dat_i   => ishw_se_m_wb_dat_o,
    mi_wb_dat_o   => ishw_se_m_wb_dat_i,
    mi_wb_adr_o   => ishw_se_m_wb_adr_i(maxAddrBit downto 0),
    mi_wb_sel_o   => ishw_se_m_wb_sel_i,
    mi_wb_cti_o   => ishw_se_m_wb_cti_i,
    mi_wb_we_o    => ishw_se_m_wb_we_i,
    mi_wb_cyc_o   => ishw_se_m_wb_cyc_i,
    mi_wb_stb_o   => ishw_se_m_wb_stb_i,
    mi_wb_ack_i   => ishw_se_m_wb_ack_o,
    mi_wb_stall_i => ishw_se_m_wb_stall_o,

    scki       => ishw_se_ckis,
    mosi       => ishw_se_dati,
    miso       => ishw_se_dato
  );

  --
  -- IO SLOT 11
  --

  slot11: ishw_slave
  port map (
    Wb_clk_i      => wb_clk_i,
    wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(11),
    wb_dat_i      => slot_write(11),
    wb_adr_i      => slot_address(11),
    wb_we_i       => slot_we(11),
    wb_cyc_i      => slot_cyc(11),
    wb_stb_i      => slot_stb(11),
    wb_ack_o      => slot_ack(11),
    wb_inta_o     => slot_interrupt(11),
    id            => slot_id(11),

    mi_wb_dat_i   => ishw_st_m_wb_dat_o,
    mi_wb_dat_o   => ishw_st_m_wb_dat_i,
    mi_wb_adr_o   => ishw_st_m_wb_adr_i(maxAddrBit downto 0),
    mi_wb_sel_o   => ishw_st_m_wb_sel_i,
    mi_wb_cti_o   => ishw_st_m_wb_cti_i,
    mi_wb_we_o    => ishw_st_m_wb_we_i,
    mi_wb_cyc_o   => ishw_st_m_wb_cyc_i,
    mi_wb_stb_o   => ishw_st_m_wb_stb_i,
    mi_wb_ack_i   => ishw_st_m_wb_ack_o,
    mi_wb_stall_i => ishw_st_m_wb_stall_o,

    scki       => ishw_st_ckis,
    mosi       => ishw_st_dati,
    miso       => ishw_st_dato
  );

  --
  -- IO SLOT 12
  --

  slot12: ishw_master
  port map (
    Wb_clk_i      => wb_clk_i,
    wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(12),
    wb_dat_i      => slot_write(12),
    wb_adr_i      => slot_address(12),
    wb_we_i       => slot_we(12),
    wb_cyc_i      => slot_cyc(12),
    wb_stb_i      => slot_stb(12),
    wb_ack_o      => slot_ack(12),
    wb_inta_o     => slot_interrupt(12),
    id            => slot_id(12),

    mi_wb_dat_i   => ishw_sw_m_wb_dat_o,
    mi_wb_dat_o   => ishw_sw_m_wb_dat_i,
    mi_wb_adr_o   => ishw_sw_m_wb_adr_i(maxAddrBit downto 0),
    mi_wb_sel_o   => ishw_sw_m_wb_sel_i,
    mi_wb_cti_o   => ishw_sw_m_wb_cti_i,
    mi_wb_we_o    => ishw_sw_m_wb_we_i,
    mi_wb_cyc_o   => ishw_sw_m_wb_cyc_i,
    mi_wb_stb_o   => ishw_sw_m_wb_stb_i,
    mi_wb_ack_i   => ishw_sw_m_wb_ack_o,
    mi_wb_stall_i => ishw_sw_m_wb_stall_o,

    txclk      => clkdiv,
    scko       => ishw_sw_ckom,
    mosi       => ishw_sw_dato,
    miso       => ishw_sw_dati
  );

  --
  -- IO SLOT 13
  --

  slot13: ishw_slave
  port map (
    Wb_clk_i      => wb_clk_i,
    wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(13),
    wb_dat_i      => slot_write(13),
    wb_adr_i      => slot_address(13),
    wb_we_i       => slot_we(13),
    wb_cyc_i      => slot_cyc(13),
    wb_stb_i      => slot_stb(13),
    wb_ack_o      => slot_ack(13),
    wb_inta_o     => slot_interrupt(13),
    id            => slot_id(13),

    mi_wb_dat_i   => ishw_wt_m_wb_dat_o,
    mi_wb_dat_o   => ishw_wt_m_wb_dat_i,
    mi_wb_adr_o   => ishw_wt_m_wb_adr_i(maxAddrBit downto 0),
    mi_wb_sel_o   => ishw_wt_m_wb_sel_i,
    mi_wb_cti_o   => ishw_wt_m_wb_cti_i,
    mi_wb_we_o    => ishw_wt_m_wb_we_i,
    mi_wb_cyc_o   => ishw_wt_m_wb_cyc_i,
    mi_wb_stb_o   => ishw_wt_m_wb_stb_i,
    mi_wb_ack_i   => ishw_wt_m_wb_ack_o,
    mi_wb_stall_i => ishw_wt_m_wb_stall_o,

    scki       => ishw_wt_ckis,
    mosi       => ishw_wt_dati,
    miso       => ishw_wt_dato
  );

  --
  -- IO SLOT 14
  --

  slot14: ishw_master
  port map (
    Wb_clk_i      => wb_clk_i,
    wb_rst_i      => wb_rst_i,
    wb_dat_o      => slot_read(14),
    wb_dat_i      => slot_write(14),
    wb_adr_i      => slot_address(14),
    wb_we_i       => slot_we(14),
    wb_cyc_i      => slot_cyc(14),
    wb_stb_i      => slot_stb(14),
    wb_ack_o      => slot_ack(14),
    wb_inta_o     => slot_interrupt(14),
    id            => slot_id(14),

    mi_wb_dat_i   => ishw_nw_m_wb_dat_o,
    mi_wb_dat_o   => ishw_nw_m_wb_dat_i,
    mi_wb_adr_o   => ishw_nw_m_wb_adr_i(maxAddrBit downto 0),
    mi_wb_sel_o   => ishw_nw_m_wb_sel_i,
    mi_wb_cti_o   => ishw_nw_m_wb_cti_i,
    mi_wb_we_o    => ishw_nw_m_wb_we_i,
    mi_wb_cyc_o   => ishw_nw_m_wb_cyc_i,
    mi_wb_stb_o   => ishw_nw_m_wb_stb_i,
    mi_wb_ack_i   => ishw_nw_m_wb_ack_o,
    mi_wb_stall_i => ishw_nw_m_wb_stall_o,

    txclk      => clkdiv,
    scko       => ishw_nw_ckom,
    mosi       => ishw_nw_dato,
    miso       => ishw_nw_dati
  );

  --
  -- IO SLOT 15 - do not use
  --

  process(gpio_spp_read, spi_pf_mosi, spi_pf_sck,
          timers_pwm,
          spi2_mosi,spi2_sck)
  begin

    gpio_spp_data <= (others => DontCareValue);

    -- PPS Outputs
    -- gpio_spp_data(0)  <= sigmadelta_spp_data(0);   -- PPS0 : SIGMADELTA DATA
    -- ppsout_info_slot(0) <= 5; -- Slot 5
    -- ppsout_info_pin(0) <= 0;  -- PPS OUT pin 0 (Channel 0)

    gpio_spp_data(1)  <= timers_pwm(0);            -- PPS1 : TIMER0
    ppsout_info_slot(1) <= 3; -- Slot 3
    ppsout_info_pin(1) <= 0;  -- PPS OUT pin 0 (TIMER 0)

    gpio_spp_data(2)  <= timers_pwm(1);            -- PPS2 : TIMER1
    ppsout_info_slot(2) <= 3; -- Slot 3
    ppsout_info_pin(2) <= 1;  -- PPS OUT pin 1 (TIMER 0)

    gpio_spp_data(3)  <= spi2_mosi;                -- PPS3 : USPI MOSI
    ppsout_info_slot(3) <= 6; -- Slot 6
    ppsout_info_pin(3) <= 0;  -- PPS OUT pin 0 (MOSI)

    gpio_spp_data(4)  <= spi2_sck;                 -- PPS4 : USPI SCK
    ppsout_info_slot(4) <= 6; -- Slot 6
    ppsout_info_pin(4) <= 1;  -- PPS OUT pin 1 (SCK)

    -- gpio_spp_data(5)  <= sigmadelta_spp_data(1);   -- PPS5 : SIGMADELTA1 DATA
    -- ppsout_info_slot(5) <= 5; -- Slot 5
    -- ppsout_info_pin(5) <= 1;  -- PPS OUT pin 0 (Channel 1)

    -- PPS inputs
    spi2_miso         <= gpio_spp_read(0);         -- PPS0 : USPI MISO
    ppsin_info_slot(0) <= 6;                    -- USPI is in slot 6
    ppsin_info_pin(0) <= 0;                     -- PPS pin of USPI is 0

  end process;

  arb: wbarb8_1
  generic map (
    ADDRESS_HIGH => maxAddrBit,
    ADDRESS_LOW => 0
  )
  port map (
    wb_clk_i      => wb_clk_i,
    wb_rst_i      => wb_rst_i,

    -- Master 0 signals

    m0_wb_dat_o   => ishw_nt_m_wb_dat_o,
    m0_wb_dat_i   => ishw_nt_m_wb_dat_i,
    m0_wb_adr_i   => ishw_nt_m_wb_adr_i,
    m0_wb_sel_i   => ishw_nt_m_wb_sel_i,
    m0_wb_cti_i   => CTI_CYCLE_CLASSIC,
    m0_wb_we_i    => ishw_nt_m_wb_we_i,
    m0_wb_cyc_i   => ishw_nt_m_wb_cyc_i,
    m0_wb_stb_i   => ishw_nt_m_wb_stb_i,
    m0_wb_ack_o   => ishw_nt_m_wb_ack_o,
    m0_wb_stall_o => ishw_nt_m_wb_stall_o,

    -- Master 1 signals

    m1_wb_dat_o   => ishw_ne_m_wb_dat_o,
    m1_wb_dat_i   => ishw_ne_m_wb_dat_i,
    m1_wb_adr_i   => ishw_ne_m_wb_adr_i,
    m1_wb_sel_i   => ishw_ne_m_wb_sel_i,
    m1_wb_cti_i   => CTI_CYCLE_CLASSIC,
    m1_wb_we_i    => ishw_ne_m_wb_we_i,
    m1_wb_cyc_i   => ishw_ne_m_wb_cyc_i,
    m1_wb_stb_i   => ishw_ne_m_wb_stb_i,
    m1_wb_ack_o   => ishw_ne_m_wb_ack_o,
    m1_wb_stall_o => ishw_ne_m_wb_stall_o,

    -- Master 2 signals

    m2_wb_dat_o   => ishw_et_m_wb_dat_o,
    m2_wb_dat_i   => ishw_et_m_wb_dat_i,
    m2_wb_adr_i   => ishw_et_m_wb_adr_i,
    m2_wb_sel_i   => ishw_et_m_wb_sel_i,
    m2_wb_cti_i   => CTI_CYCLE_CLASSIC,
    m2_wb_we_i    => ishw_et_m_wb_we_i,
    m2_wb_cyc_i   => ishw_et_m_wb_cyc_i,
    m2_wb_stb_i   => ishw_et_m_wb_stb_i,
    m2_wb_ack_o   => ishw_et_m_wb_ack_o,
    m2_wb_stall_o => ishw_et_m_wb_stall_o,

    -- Master 3 signals

    m3_wb_dat_o   => ishw_se_m_wb_dat_o,
    m3_wb_dat_i   => ishw_se_m_wb_dat_i,
    m3_wb_adr_i   => ishw_se_m_wb_adr_i,
    m3_wb_sel_i   => ishw_se_m_wb_sel_i,
    m3_wb_cti_i   => CTI_CYCLE_CLASSIC,
    m3_wb_we_i    => ishw_se_m_wb_we_i,
    m3_wb_cyc_i   => ishw_se_m_wb_cyc_i,
    m3_wb_stb_i   => ishw_se_m_wb_stb_i,
    m3_wb_ack_o   => ishw_se_m_wb_ack_o,
    m3_wb_stall_o => ishw_se_m_wb_stall_o,

    -- Master 4 signals

    m4_wb_dat_o   => ishw_st_m_wb_dat_o,
    m4_wb_dat_i   => ishw_st_m_wb_dat_i,
    m4_wb_adr_i   => ishw_st_m_wb_adr_i,
    m4_wb_sel_i   => ishw_st_m_wb_sel_i,
    m4_wb_cti_i   => CTI_CYCLE_CLASSIC,
    m4_wb_we_i    => ishw_st_m_wb_we_i,
    m4_wb_cyc_i   => ishw_st_m_wb_cyc_i,
    m4_wb_stb_i   => ishw_st_m_wb_stb_i,
    m4_wb_ack_o   => ishw_st_m_wb_ack_o,
    m4_wb_stall_o => ishw_st_m_wb_stall_o,

    -- Master 5 signals

    m5_wb_dat_o   => ishw_sw_m_wb_dat_o,
    m5_wb_dat_i   => ishw_sw_m_wb_dat_i,
    m5_wb_adr_i   => ishw_sw_m_wb_adr_i,
    m5_wb_sel_i   => ishw_sw_m_wb_sel_i,
    m5_wb_cti_i   => CTI_CYCLE_CLASSIC,
    m5_wb_we_i    => ishw_sw_m_wb_we_i,
    m5_wb_cyc_i   => ishw_sw_m_wb_cyc_i,
    m5_wb_stb_i   => ishw_sw_m_wb_stb_i,
    m5_wb_ack_o   => ishw_sw_m_wb_ack_o,
    m5_wb_stall_o => ishw_sw_m_wb_stall_o,

    -- Master 6 signals

    m6_wb_dat_o   => ishw_wt_m_wb_dat_o,
    m6_wb_dat_i   => ishw_wt_m_wb_dat_i,
    m6_wb_adr_i   => ishw_wt_m_wb_adr_i,
    m6_wb_sel_i   => ishw_wt_m_wb_sel_i,
    m6_wb_cti_i   => CTI_CYCLE_CLASSIC,
    m6_wb_we_i    => ishw_wt_m_wb_we_i,
    m6_wb_cyc_i   => ishw_wt_m_wb_cyc_i,
    m6_wb_stb_i   => ishw_wt_m_wb_stb_i,
    m6_wb_ack_o   => ishw_wt_m_wb_ack_o,
    m6_wb_stall_o => ishw_wt_m_wb_stall_o,

    -- Master 7 signals

    m7_wb_dat_o   => ishw_nw_m_wb_dat_o,
    m7_wb_dat_i   => ishw_nw_m_wb_dat_i,
    m7_wb_adr_i   => ishw_nw_m_wb_adr_i,
    m7_wb_sel_i   => ishw_nw_m_wb_sel_i,
    m7_wb_cti_i   => CTI_CYCLE_CLASSIC,
    m7_wb_we_i    => ishw_nw_m_wb_we_i,
    m7_wb_cyc_i   => ishw_nw_m_wb_cyc_i,
    m7_wb_stb_i   => ishw_nw_m_wb_stb_i,
    m7_wb_ack_o   => ishw_nw_m_wb_ack_o,
    m7_wb_stall_o => ishw_nw_m_wb_stall_o,


    -- Slave signals

    s0_wb_dat_i   => m_wb_dat_o,
    s0_wb_dat_o   => m_wb_dat_i,
    s0_wb_adr_o   => m_wb_adr_i,
    s0_wb_sel_o   => m_wb_sel_i,
    s0_wb_cti_o   => open,
    s0_wb_we_o    => m_wb_we_i,
    s0_wb_cyc_o   => m_wb_cyc_i,
    s0_wb_stb_o   => m_wb_stb_i,
    s0_wb_ack_i   => m_wb_ack_o,
    s0_wb_stall_i => m_wb_stall_o
  );



end behave;
