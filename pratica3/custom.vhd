-- ================================================================================ --
-- NEORV32 CPU - Custom Functions Subsystem (CFS)                                   --
-- -------------------------------------------------------------------------------- --
-- Acelerador Paralelo (10 Vias) 24.8                                               --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cfs is
  generic (
    CFS_CONFIG   : std_ulogic_vector(31 downto 0) := (others => '0'); 
    CFS_IN_SIZE  : natural := 256; 
    CFS_OUT_SIZE : natural := 256  
  );
  port (
    clk_i       : in  std_ulogic;
    rstn_i      : in  std_ulogic;
    bus_req_i   : in  bus_req_t;
    bus_rsp_o   : out bus_rsp_t;
    clkgen_en_o : out std_ulogic;
    clkgen_i    : in  std_ulogic_vector(7 downto 0) := (others => '0');
    irq_o       : out std_ulogic;
    cfs_in_i    : in  std_ulogic_vector(CFS_IN_SIZE-1 downto 0) := (others => '0');
    cfs_out_o   : out std_ulogic_vector(CFS_OUT_SIZE-1 downto 0)
  );
end neorv32_cfs;

architecture neorv32_cfs_rtl of neorv32_cfs is

  -- Registradores para a Matriz B (Agora são 10 valores!)
  signal reg_b0, reg_b1, reg_b2, reg_b3, reg_b4 : signed(31 downto 0);
  signal reg_b5, reg_b6, reg_b7, reg_b8, reg_b9 : signed(31 downto 0);
  
  -- 10 Acumuladores de 64 bits
  signal mac0, mac1, mac2, mac3, mac4 : signed(63 downto 0);
  signal mac5, mac6, mac7, mac8, mac9 : signed(63 downto 0);

begin

  process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      reg_b0 <= (others => '0'); reg_b1 <= (others => '0'); reg_b2 <= (others => '0');
      reg_b3 <= (others => '0'); reg_b4 <= (others => '0'); reg_b5 <= (others => '0');
      reg_b6 <= (others => '0'); reg_b7 <= (others => '0'); reg_b8 <= (others => '0');
      reg_b9 <= (others => '0');
      
      mac0 <= (others => '0'); mac1 <= (others => '0'); mac2 <= (others => '0');
      mac3 <= (others => '0'); mac4 <= (others => '0'); mac5 <= (others => '0');
      mac6 <= (others => '0'); mac7 <= (others => '0'); mac8 <= (others => '0');
      mac9 <= (others => '0');
      
      bus_rsp_o.ack  <= '0';
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      
    elsif rising_edge(clk_i) then
      bus_rsp_o.ack  <= '0';
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');

      if (bus_req_i.stb = '1') then
        
        -- ESCRITA (CPU -> Hardware)
        if (bus_req_i.rw = '1') then
          bus_rsp_o.ack <= '1';
          
          -- addr(5 downto 2) dá-nos 16 endereços possíveis (0 a 15)
          case bus_req_i.addr(5 downto 2) is
            when "0000" => reg_b0 <= signed(bus_req_i.data); -- REG[0]
            when "0001" => reg_b1 <= signed(bus_req_i.data); -- REG[1]
            when "0010" => reg_b2 <= signed(bus_req_i.data); -- REG[2]
            when "0011" => reg_b3 <= signed(bus_req_i.data); -- REG[3]
            when "0100" => reg_b4 <= signed(bus_req_i.data); -- REG[4]
            when "0101" => reg_b5 <= signed(bus_req_i.data); -- REG[5]
            when "0110" => reg_b6 <= signed(bus_req_i.data); -- REG[6]
            when "0111" => reg_b7 <= signed(bus_req_i.data); -- REG[7]
            when "1000" => reg_b8 <= signed(bus_req_i.data); -- REG[8]
            when "1001" => reg_b9 <= signed(bus_req_i.data); -- REG[9]
            
            when "1010" => -- REG[10]: GATILHO: Multiplica e Acumula as 10 vias
              mac0 <= mac0 + (signed(bus_req_i.data) * reg_b0);
              mac1 <= mac1 + (signed(bus_req_i.data) * reg_b1);
              mac2 <= mac2 + (signed(bus_req_i.data) * reg_b2);
              mac3 <= mac3 + (signed(bus_req_i.data) * reg_b3);
              mac4 <= mac4 + (signed(bus_req_i.data) * reg_b4);
              mac5 <= mac5 + (signed(bus_req_i.data) * reg_b5);
              mac6 <= mac6 + (signed(bus_req_i.data) * reg_b6);
              mac7 <= mac7 + (signed(bus_req_i.data) * reg_b7);
              mac8 <= mac8 + (signed(bus_req_i.data) * reg_b8);
              mac9 <= mac9 + (signed(bus_req_i.data) * reg_b9);
              
            when "1011" => -- REG[11]: Zera todos os 10 acumuladores
              mac0 <= (others => '0'); mac1 <= (others => '0'); mac2 <= (others => '0');
              mac3 <= (others => '0'); mac4 <= (others => '0'); mac5 <= (others => '0');
              mac6 <= (others => '0'); mac7 <= (others => '0'); mac8 <= (others => '0');
              mac9 <= (others => '0');
              
            when others => null;
          end case;
        
        -- LEITURA (Hardware -> CPU)
        else
          bus_rsp_o.ack <= '1';
          case bus_req_i.addr(5 downto 2) is
            when "0000" => bus_rsp_o.data <= std_ulogic_vector(mac0(39 downto 8));
            when "0001" => bus_rsp_o.data <= std_ulogic_vector(mac1(39 downto 8));
            when "0010" => bus_rsp_o.data <= std_ulogic_vector(mac2(39 downto 8));
            when "0011" => bus_rsp_o.data <= std_ulogic_vector(mac3(39 downto 8));
            when "0100" => bus_rsp_o.data <= std_ulogic_vector(mac4(39 downto 8));
            when "0101" => bus_rsp_o.data <= std_ulogic_vector(mac5(39 downto 8));
            when "0110" => bus_rsp_o.data <= std_ulogic_vector(mac6(39 downto 8));
            when "0111" => bus_rsp_o.data <= std_ulogic_vector(mac7(39 downto 8));
            when "1000" => bus_rsp_o.data <= std_ulogic_vector(mac8(39 downto 8));
            when "1001" => bus_rsp_o.data <= std_ulogic_vector(mac9(39 downto 8));
            when others => bus_rsp_o.data <= (others => '0');
          end case;
        end if;

      end if;
    end if;
  end process;

  clkgen_en_o <= '0';
  irq_o       <= '0';
  cfs_out_o   <= (others => '0');

end neorv32_cfs_rtl;