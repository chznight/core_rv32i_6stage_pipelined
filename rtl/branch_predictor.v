// Copyright (c) 2026 chznight
// SPDX-License-Identifier: MIT

module branch_predictor #(
    parameter PHT_BITS = 9,
    parameter BTB_BITS = 8,
    parameter PHT_LEN = 1 << PHT_BITS,
    parameter BTB_LEN = 1 << BTB_BITS,
    parameter JMP_TYPE = 2'b10,
    parameter BRANCH_TYPE = 2'b01
)(
    input wire clk,
    input wire reset,
    input wire [31:0] pc,
    output reg predict_taken,
    output wire [31:0] predict_branch_target,
    input wire [31:0] update_predictor_pc,
    input wire [31:0] update_predictor_branch_target,
    input wire update_pht_en,
    input wire update_btb_en,
    input wire update_predictor_taken,
    input wire predictor_missed,
    input wire predictor_target_missed,
    input wire [1:0] instruction_type,
    output wire [PHT_BITS-1:0] branch_gshare_index,
    input wire [PHT_BITS-1:0] predictor_update_gshare_index
);

    localparam BTB_TAG_BITS = 30 - BTB_BITS;

    reg [31:0] predictor_miss_counter;
    reg [31:0] predictor_target_miss_counter;
    reg [31:0] total_branches_counter;

    // ============================================================
    // Gshare state
    // ============================================================
    reg [PHT_BITS-1:0] global_history;
    reg [1:0] counter_table [0:PHT_LEN-1];

    // ============================================================
    // BTB state
    // ============================================================
    reg [31:0] branch_target_table [0:BTB_LEN-1];
    reg [BTB_TAG_BITS-1:0] branch_tag_table [0:BTB_LEN-1];
    reg branch_target_table_valid [0:BTB_LEN-1];
    reg [1:0] instruction_type_table [0:BTB_LEN-1];

    integer i;

    // ============================================================
    // Index calculation
    // ============================================================
    wire [PHT_BITS-1:0] pc_pht_index;
    wire [PHT_BITS-1:0] gshare_index;
    wire [PHT_BITS-1:0] update_gshare_index;

    wire [BTB_BITS-1:0] pc_btb_index;
    wire [BTB_BITS-1:0] update_btb_index;

    wire [BTB_TAG_BITS-1:0] pc_btb_tag;
    wire [BTB_TAG_BITS-1:0] update_btb_tag;

    assign pc_pht_index = pc[PHT_BITS+1:2];
    assign pc_btb_index = pc[BTB_BITS+1:2];

    assign update_btb_index = update_predictor_pc[BTB_BITS+1:2];

    assign pc_btb_tag = pc[31:BTB_BITS+2];
    assign update_btb_tag = update_predictor_pc[31:BTB_BITS+2];

    // Gshare index = PHT PC bits XOR global history
    assign gshare_index = pc_pht_index ^ global_history;
    assign update_gshare_index = predictor_update_gshare_index;
    assign branch_gshare_index = gshare_index;

    wire [1:0] counter_value;
    wire [1:0] current_value_before_update;

    assign counter_value = counter_table[gshare_index];
    assign current_value_before_update = counter_table[update_gshare_index];

    reg [1:0] next_counter_value;

    // ============================================================
    // 2-bit saturating counter update logic
    // ============================================================

    always @(*) begin
        if (update_predictor_taken) begin
            if (current_value_before_update == 2'b11)
                next_counter_value = 2'b11;
            else
                next_counter_value = current_value_before_update + 2'b01;
        end else begin
            if (current_value_before_update == 2'b00)
                next_counter_value = 2'b00;
            else
                next_counter_value = current_value_before_update - 2'b01;
        end
    end

    // ============================================================
    // Prediction logic
    // ============================================================

    always @(*) begin
        predict_taken = 1'b0;
        if (branch_target_table_valid[pc_btb_index] && branch_tag_table[pc_btb_index] == pc_btb_tag) begin
            if (instruction_type_table[pc_btb_index] == JMP_TYPE) begin
                predict_taken = 1'b1;
            end else if (instruction_type_table[pc_btb_index] == BRANCH_TYPE) begin
                if (counter_value >= 2'b10) begin
                    predict_taken = 1'b1;
                end
            end
        end
    end
    // Target comes from the smaller PC-indexed BTB
    assign predict_branch_target = branch_target_table[pc_btb_index];

    // ============================================================
    // Predictor table update
    // ============================================================

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            global_history <= {PHT_BITS{1'b0}};
            for (i = 0; i < PHT_LEN; i = i + 1) begin
                counter_table[i] <= 2'b01;
            end
            for (i = 0; i < BTB_LEN; i = i + 1) begin
                branch_target_table_valid[i] <= 1'b0;
            end
        end else begin
            // Update PHT counter only when a conditional branch resolves.
            // Update global control-flow history for branches and jumps.
            if (update_pht_en) begin
                counter_table[update_gshare_index] <= next_counter_value;
                global_history <= {global_history[PHT_BITS-2:0], update_predictor_taken};
            end

            // Mark BTB entry valid only when storing a taken target.
            if (update_predictor_taken && update_btb_en) begin
                branch_target_table_valid[update_btb_index] <= 1'b1;
            end
        end
    end

    // ============================================================
    // BTB target/tag/type update
    // ============================================================

    always @(posedge clk) begin
        if (update_btb_en && update_predictor_taken) begin
            branch_target_table[update_btb_index] <= update_predictor_branch_target;
            branch_tag_table[update_btb_index] <= update_btb_tag;
            instruction_type_table[update_btb_index] <= instruction_type;
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            predictor_miss_counter <= 32'b0;
            predictor_target_miss_counter <= 32'b0;
        end else if (update_pht_en | update_btb_en) begin
            if (predictor_missed) begin
                predictor_miss_counter <= predictor_miss_counter + 1;
            end

            if (predictor_target_missed) begin
                predictor_target_miss_counter <= predictor_target_miss_counter + 1;
            end
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            total_branches_counter <= 32'b0;
        end else if (update_pht_en | update_btb_en) begin
            total_branches_counter <= total_branches_counter + 1;
        end
    end

endmodule