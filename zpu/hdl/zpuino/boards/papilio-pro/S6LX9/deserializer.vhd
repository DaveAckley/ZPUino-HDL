library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.txt_util.all;

entity deserializer is
  port (
    clk: in std_logic;
    data:in std_logic;
    datavalid: out std_logic;
    dataout: out std_logic_vector(7 downto 0);
    clkout: out std_logic;
    framedetected: out std_logic;
    idledetected: out std_logic
  );
end entity deserializer;

architecture sim of deserializer is
  signal inframe: std_logic := '0';
  signal shreg, datareg: std_logic_vector(7 downto 0);
  signal issync: std_logic;
  signal isidle: std_logic;
  signal isescape: std_logic;
  signal cnt: integer range 0 to 7;
  signal inescape: std_logic := '0';
  signal valid: std_logic;
begin

  datavalid<=valid;
  clkout <= clk;
  dataout <= datareg;

  issync <= '1' when shreg(6 downto 0)="0111111" and data='1' else '0';
  isidle <= '1' when shreg(6 downto 0)="0111111" and data='0' else '0';
  isescape <= '1' when shreg(5 downto 0)="011111" and data='0' else '0';

  process(clk)
  begin
    if rising_edge(clk) then
      valid <= '0';
      framedetected<='0';
      if inescape='1' then
        inescape<='0';
        shreg(0)<=data;
        shreg(7 downto 1) <= shreg(6 downto 0);
        datareg(0)<=data;
        datareg(7 downto 1) <= datareg(6 downto 0);
        if cnt=0 then
          valid <= '1';
          cnt<=7;
        else
          cnt<=cnt - 1;
        end if;
      else
        if issync='1' then
          -- We see a sync pattern
          cnt <= 7;
          inframe <= '1';
          --report "Start/end FRAME";
          framedetected<='1';
        elsif isidle='1' then
          -- Nothing to do.
          --report "Idle TAG";
          idledetected<='1';
          cnt<=7;
         else
           if isescape='0' then
            if cnt=0 then
             if issync='0' then
              valid <= '1';
             end if;
             cnt<=7;
           else
            cnt<=cnt - 1;
           end if;
           end if;
         end if;

        inescape<='0';

        if isescape='1' then
          inescape<='1';
        end if;

        shreg(0)<=data;
        shreg(7 downto 1) <= shreg(6 downto 0);

        if isescape='0' then
          datareg(0)<=data;
          datareg(7 downto 1) <= datareg(6 downto 0);
        end if;
      end if;
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if valid='1' then
        --report "DATA: " & str(datareg);
      end if;
    end if;
  end process;

end sim;
