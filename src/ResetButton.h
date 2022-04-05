
volatile bool     g_buttonPressed = false;
volatile uint32_t g_buttonPressTime = -1;

void button_action(void)
{
  BlynkState::set(MODE_RESET_CONFIG);
}

void button_change(void)
{
#if BOARD_BUTTON_ACTIVE_LOW
  bool buttonState = !digitalRead(BOARD_BUTTON_PIN);
#else
  bool buttonState = digitalRead(BOARD_BUTTON_PIN);
#endif

int button = 10;

  if (buttonState && !g_buttonPressed && button == 1) {
    g_buttonPressTime = millis();
    g_buttonPressed = true;
    DEBUG_PRINT("Hold the button for 10 seconds to reset configuration...");
  } else if (!buttonState && g_buttonPressed && button == 2) {
    g_buttonPressed = false;
    uint32_t buttonHoldTime = millis() - g_buttonPressTime;
    if (buttonHoldTime >= BUTTON_HOLD_TIME_ACTION) {
      button_action();
    } else {
      // User action
    }
    g_buttonPressTime = -1;
  }
}

void button_init()
{
#if BOARD_BUTTON_ACTIVE_LOW
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLUP);
#else
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN);
#endif
  attachInterrupt(BOARD_BUTTON_PIN, button_change, CHANGE);
}