#include "QuantumOdom/OdomDebug.hpp"

OdomDebug::OdomDebug(lv_obj_t* parent)
    : OdomDebug(parent, lv_obj_get_style(parent)->body.main_color) {}


OdomDebug::OdomDebug(lv_obj_t* parent, lv_color_t mainColor)
    : container(lv_obj_create(parent, NULL)) {

    lv_obj_set_size(container, lv_obj_get_width(parent), lv_obj_get_height(parent));
    lv_obj_align(container, NULL, LV_ALIGN_CENTER, 0, 0);

    /**
     * Container Style
     */
    lv_style_copy(&cStyle, &lv_style_plain_color);
    cStyle.body.main_color = mainColor;
    cStyle.body.grad_color = mainColor;
    cStyle.body.border.width = 0;
    cStyle.body.radius = 0;
    lv_obj_set_style(container, &cStyle);

    /**
    * Field
    */
    lv_obj_t* field = lv_obj_create(container, NULL);
    fieldDim = std::min(lv_obj_get_width(container), lv_obj_get_height(container));
    lv_obj_set_size(field, fieldDim, fieldDim);
    lv_obj_align(field, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);

    /**
     * Field Style
     */
    lv_style_copy(&fStyle, &cStyle);
    fStyle.body.main_color = LV_COLOR_WHITE;
    fStyle.body.grad_color = LV_COLOR_WHITE;
    lv_obj_set_style(field, &fStyle);

    /**
     * Tile Styles
     */
    lv_style_copy(&grey, &lv_style_plain);
    grey.body.main_color = LV_COLOR_HEX(0x828F8F);
    grey.body.grad_color = LV_COLOR_HEX(0x828F8F);
    grey.body.border.width = 1;
    grey.body.radius = 0;
    grey.body.border.color = LV_COLOR_WHITE;

    lv_style_copy(&red, &grey);
    red.body.main_color = LV_COLOR_HEX(0xD42630);
    red.body.grad_color = LV_COLOR_HEX(0xD42630);
    lv_style_copy(&blue, &grey);
    blue.body.main_color = LV_COLOR_HEX(0x0077C9);
    blue.body.grad_color = LV_COLOR_HEX(0x0077C9);

    /**
     * Tile Layout
     */
    std::vector<std::vector<lv_style_t*>> tileData = {
      {&grey, &grey ,&grey, &grey, &grey, &grey},
      {&grey ,&grey, &grey, &grey, &grey, &grey},
      {&grey, &grey, &grey, &grey, &grey, &grey},
      {&grey, &grey, &grey, &grey, &grey, &grey},
      {&grey, &grey, &grey, &grey, &grey, &grey},
      {&grey, &grey, &grey, &grey, &grey, &grey}
    };

    double tileDim = fieldDim / tileData.size(); // tile dimention

    /**
     * Create tile matrix, register callbacks, assign each tile an ID
     */
    for (size_t y = 0; y < 6; y++) {
        for (size_t x = 0; x < 6; x++) {
            lv_obj_t* tileObj = lv_btn_create(field, NULL);
            lv_obj_set_pos(tileObj, x * tileDim, y * tileDim);
            lv_obj_set_size(tileObj, tileDim, tileDim);
            lv_btn_set_action(tileObj, LV_BTN_ACTION_CLICK, tileAction);
            lv_obj_set_free_num(tileObj, y * 6 + x);
            lv_obj_set_free_ptr(tileObj, this);
            lv_btn_set_toggle(tileObj, false);
            lv_btn_set_style(tileObj, LV_BTN_STYLE_PR, tileData[y][x]);
            lv_btn_set_style(tileObj, LV_BTN_STYLE_REL, tileData[y][x]);
        }
    }

    /**
     * Robot point using lvgl led
     */
    led = lv_led_create(field, NULL);
    lv_led_on(led);
    lv_obj_set_size(led, fieldDim / 15, fieldDim / 15);

    lv_style_copy(&ledStyle, &lv_style_plain);
    ledStyle.body.radius = LV_RADIUS_CIRCLE;
    ledStyle.body.main_color = mainColor;
    ledStyle.body.grad_color = mainColor;
    ledStyle.body.border.color = LV_COLOR_WHITE;
    ledStyle.body.border.width = 2;
    ledStyle.body.border.opa = LV_OPA_100;
    lv_obj_set_style(led, &ledStyle);


    /**
     * Robot line
     */
    line = lv_line_create(field, NULL);
    lv_line_set_points(line, linePoints.data(), linePoints.size());
    lv_obj_set_pos(line, 0, 0);

    lineWidth = 3;
    lineLength = fieldDim / 6;

    lv_style_copy(&lineStyle, &lv_style_plain);
    lineStyle.line.width = 3;
    lineStyle.line.opa = LV_OPA_100;
    lineStyle.line.color = mainColor;
    lv_obj_set_style(line, &lineStyle);

    /**
     * Status Label
     */
    statusLabel = lv_label_create(container, NULL);
    lv_style_copy(&textStyle, &lv_style_plain);
    textStyle.text.color = LV_COLOR_WHITE;
    textStyle.text.opa = LV_OPA_100;
    lv_obj_set_style(statusLabel, &textStyle);
    lv_label_set_text(statusLabel, "No Odom Data Provided");
    lv_obj_align(statusLabel, container, LV_ALIGN_CENTER, -lv_obj_get_width(container) / 2 + (lv_obj_get_width(container) - fieldDim) / 2, 0);

    /**
    * Reset Button
    */
    {
        lv_obj_t* btn = lv_btn_create(container, NULL);
        lv_obj_set_size(btn, 100, 40);
        lv_obj_align(btn, NULL, LV_ALIGN_IN_TOP_MID, -lv_obj_get_width(container) / 2 + (lv_obj_get_width(container) - fieldDim) / 2, 0);
        lv_obj_set_free_ptr(btn, this);
        lv_btn_set_action(btn, LV_BTN_ACTION_PR, resetAction);

        /**
         * Button Style
         */
        lv_style_copy(&resetRel, &lv_style_btn_tgl_rel);
        resetRel.body.main_color = mainColor;
        resetRel.body.grad_color = mainColor;
        resetRel.body.border.color = LV_COLOR_WHITE;
        resetRel.body.border.width = 2;
        resetRel.body.border.opa = LV_OPA_100;
        resetRel.body.radius = 2;
        resetRel.text.color = LV_COLOR_WHITE;

        lv_style_copy(&resetPr, &resetRel);
        resetPr.body.main_color = LV_COLOR_WHITE;
        resetPr.body.grad_color = LV_COLOR_WHITE;
        resetPr.text.color = mainColor;

        lv_btn_set_style(btn, LV_BTN_STYLE_REL, &resetRel);
        lv_btn_set_style(btn, LV_BTN_STYLE_PR, &resetPr);

        /**
        * Reset Button Label
        */
        lv_obj_t* label = lv_label_create(btn, NULL);
        lv_obj_set_style(label, &textStyle);
        lv_label_set_text(label, "Reset");
    }

}

OdomDebug::~OdomDebug() {
    lv_obj_del(container);
}



void OdomDebug::setStateCallback(std::function<void(State state)> callback) {
    stateFnc = callback;
}


void OdomDebug::setResetCallback(std::function<void()> callback) {
    resetFnc = callback;
}


const double PI = 3.14159;

void OdomDebug::setData(State state, Sensor_vals sensors) {

    // position in court units - 12 ft, 144 inches
    double c_x = state.x / 144;
    double c_y = (144 - state.y) / 144;
    double c_theta = state.theta;

    // place point on field
    lv_obj_set_pos(led, (c_x * fieldDim) - lv_obj_get_width(led) / 2, (c_y * fieldDim) - lv_obj_get_height(led) / 2 - 1);

    // move start and end of line
    linePoints[0] = { (int16_t)((c_x * fieldDim)), (int16_t)((c_y * fieldDim) - (lineWidth / 2)) };
    double newY = lineLength * cos(c_theta);
    double newX = lineLength * sin(c_theta);
    linePoints[1] = { (int16_t)(newX + linePoints[0].x), (int16_t)(-newY + linePoints[0].y) };

    lv_line_set_points(line, linePoints.data(), linePoints.size());
    lv_obj_invalidate(line);

    std::string text =
        "X_in: " + std::to_string(state.x) + "\n" +
        "Y_in: " + std::to_string(state.y) + "\n" +
        "Theta_deg: " + std::to_string(state.theta * 180 / PI) + "\n" +
        "Left: " + std::to_string(sensors.left) + "\n" +
        "Right: " + std::to_string(sensors.right);
    if (sensors.hasMiddle) {
        text = text + "\n" + "Middle: " + std::to_string(sensors.middle);
    }

    lv_label_set_text(statusLabel, text.c_str());
    lv_obj_align(statusLabel, container, LV_ALIGN_CENTER, -lv_obj_get_width(container) / 2 + (lv_obj_get_width(container) - fieldDim) / 2, 0);
}


lv_res_t OdomDebug::tileAction(lv_obj_t* tileObj) {
    OdomDebug* that = static_cast<OdomDebug*>(lv_obj_get_free_ptr(tileObj));
    int num = lv_obj_get_free_num(tileObj);
    int y = num / 6;
    int x = num - y * 6;
    if (that->stateFnc) that->stateFnc({ x * 24 + 12, 144 - y * 24 - 12, 0 });
    else std::cout << "OdomDebug: No tile action callback provided";
    return LV_RES_OK;
}


lv_res_t OdomDebug::resetAction(lv_obj_t* btn) {
    OdomDebug* that = static_cast<OdomDebug*>(lv_obj_get_free_ptr(btn));
    if (that->resetFnc) that->resetFnc();
    else std::cout << "OdomDebug: No reset action callback provided";
    return LV_RES_OK;
}