module cronometro (
    input        CLOCK_50, // Clock 50MHz (PIN_Y2)
    input  [2:0] KEY,      // KEY0: Reset, KEY1: Start, KEY2: Pause
    output [6:0] HEX0,     // ms Unidade (1ms)
    output [6:0] HEX1,     // ms Dezena  (10ms)
    output [6:0] HEX2,     // ms Centena (100ms)
    output [6:0] HEX3,     // Seg Unidade (1s)
    output [6:0] HEX4      // Seg Dezena  (10s)
);

    // --- Inicialização para Simulação ---
    reg [31:0] clk_count = 32'd0;
    reg        tick_1ms  = 1'b0;
    reg        running   = 1'b0;
    
    reg [3:0] ms_uni  = 4'd0;
    reg [3:0] ms_dez  = 4'd0;
    reg [3:0] ms_cen  = 4'd0;
    reg [3:0] seg_uni = 4'd0;
    reg [3:0] seg_dez = 4'd0;

    wire start_btn, pause_btn, reset_n;
    assign reset_n = KEY[0];

    // --- 1. Debouncers  ---
    debouncer db_st (CLOCK_50, ~KEY[1], start_btn);
    debouncer db_ps (CLOCK_50, ~KEY[2], pause_btn);

    // --- 2. Divisor de Clock (50MHz para 1kHz = 1ms) ---
    // 50.000.000 / 1.000 = 50.000 ciclos
    // 49999 para a placa e 49 para o ModelSim
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            clk_count <= 32'd0;
            tick_1ms  <= 1'b0;
        end else if (clk_count == 32'd49999) begin
            clk_count <= 32'd0;
            tick_1ms  <= 1'b1;
        end else begin
            clk_count <= clk_count + 1'b1;
            tick_1ms  <= 1'b0;
        end
    end

    // --- 3. Máquina de Estados ---
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) running <= 1'b0;
        else if (start_btn) running <= 1'b1;
        else if (pause_btn) running <= 1'b0;
    end

    // --- 4. Lógica de Contagem BCD ---
    always @(posedge CLOCK_50 or negedge reset_n) begin
        if (!reset_n) begin
            ms_uni <= 0; ms_dez <= 0; ms_cen <= 0;
            seg_uni <= 0; seg_dez <= 0;
        end else if (tick_1ms && running) begin
            if (ms_uni == 9) begin
                ms_uni <= 0;
                if (ms_dez == 9) begin
                    ms_dez <= 0;
                    if (ms_cen == 9) begin
                        ms_cen <= 0;
                        if (seg_uni == 9) begin
                            seg_uni <= 0;
                            if (seg_dez == 9) seg_dez <= 0;
                            else seg_dez <= seg_dez + 1;
                        end else seg_uni <= seg_uni + 1;
                    end else ms_cen <= ms_cen + 1;
                end else ms_dez <= ms_dez + 1;
            end else ms_uni <= ms_uni + 1;
        end
    end

    // --- 5. Decodificadores ---
    dec7seg d0 (ms_uni,  HEX0);
    dec7seg d1 (ms_dez,  HEX1);
    dec7seg d2 (ms_cen,  HEX2);
    dec7seg d3 (seg_uni, HEX3);
    dec7seg d4 (seg_dez, HEX4);

endmodule


module debouncer (input clk, btn_in, output reg btn_out);
    reg [19:0] count = 0;
    reg state = 0;
    always @(posedge clk) begin
        if (btn_in != state) begin state <= btn_in; count <= 0; end
        else if (count < 1000000) count <= count + 1;
        else btn_out <= state;
    end
endmodule

module dec7seg (input [3:0] bcd, output reg [6:0] seg);
    always @(*) begin
        case (bcd)
            4'h0: seg = 7'b1000000; 4'h1: seg = 7'b1111001;
            4'h2: seg = 7'b0100100; 4'h3: seg = 7'b0110000;
            4'h4: seg = 7'b0011001; 4'h5: seg = 7'b0010010;
            4'h6: seg = 7'b0000010; 4'h7: seg = 7'b1111000;
            4'h8: seg = 7'b0000000; 4'h9: seg = 7'b0010000;
            default: seg = 7'b1111111;
        endcase
    end
endmodule