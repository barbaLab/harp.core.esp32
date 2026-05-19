#ifndef HARP_C_APP_H
#define HARP_C_APP_H
#include <harp_core.h>
#include <core_registers.h>
#include <reg_types.h>


/**
 * Status LED application registers.
 *
 * Harp application registers must be exposed at APP_REG_START_ADDRESS or above.
 * These are app-relative indices 0..3, so the wire/register addresses are:
 *   LED_MODE        = APP_REG_START_ADDRESS + 0 = 32
 *   LED_COLOR_RED   = APP_REG_START_ADDRESS + 1 = 33
 *   LED_COLOR_GREEN = APP_REG_START_ADDRESS + 2 = 34
 *   LED_COLOR_BLUE  = APP_REG_START_ADDRESS + 3 = 35
 *
 * The originally proposed addresses 18..21 are not used because this core
 * defines core registers at 0..17 and APP_REG_START_ADDRESS as 32.
 */
namespace StatusLedAppRegs
{
    constexpr uint8_t LED_MODE_REL = 0;
    constexpr uint8_t LED_COLOR_RED_REL = 1;
    constexpr uint8_t LED_COLOR_GREEN_REL = 2;
    constexpr uint8_t LED_COLOR_BLUE_REL = 3;

    constexpr uint8_t LED_MODE = APP_REG_START_ADDRESS + LED_MODE_REL;
    constexpr uint8_t LED_COLOR_RED = APP_REG_START_ADDRESS + LED_COLOR_RED_REL;
    constexpr uint8_t LED_COLOR_GREEN = APP_REG_START_ADDRESS + LED_COLOR_GREEN_REL;
    constexpr uint8_t LED_COLOR_BLUE = APP_REG_START_ADDRESS + LED_COLOR_BLUE_REL;

    constexpr size_t REGISTER_COUNT = 4;

#pragma pack(push, 1)
    struct RegValues
    {
        volatile uint8_t R_LED_MODE;
        volatile uint8_t R_LED_COLOR_RED;
        volatile uint8_t R_LED_COLOR_GREEN;
        volatile uint8_t R_LED_COLOR_BLUE;
    };
#pragma pack(pop)

    extern RegValues values;
    extern RegSpecs specs[REGISTER_COUNT];
    extern RegFnPair functions[REGISTER_COUNT];

    void update();
    void reset();

    void write_led_mode(msg_t& msg);
    void write_led_color_component(msg_t& msg);
}

/**
 * \brief Harp C-style App that handles core behaviors in addition t
*   reads/writes to app-specific registers.
*   Implemented as a singleton to simplify attaching interrupt callbacks
*   (and since you can only have one per device).
 */
class HarpCApp: public HarpCore
{

// Make constructor private to prevent creating instances outside of init().
private:
/**
 * \brief constructor
 * \param app_reg_values pointer to struct containing registers.
 * \param app_reg_specs array of reg specs, indexed by app register address.
 * \param app_register_count number of app registers
 * \param reg_fns array of RegFnPairs {read fn ptr, write fn ptr}, indexed by
 *  register address.
 * \param app_reg_count number of app registers.
 * \param update_fn pointer to function that will be called periodically to
 *  update the app state.
 * \param reset_fn pointer to function that will reset the app state.
 */
    HarpCApp(uint16_t who_am_i,
             uint8_t hw_version_major, uint8_t hw_version_minor,
             uint8_t assembly_version,
             uint8_t harp_version_major, uint8_t harp_version_minor,
             uint8_t fw_version_major, uint8_t fw_version_minor,
             uint16_t serial_number, const char name[],
             const uint8_t tag[],
             void* app_reg_values, RegSpecs* app_reg_specs,
             RegFnPair* reg_fns, size_t app_reg_count,
             void (* update_fn)(void), void (* reset_fn)(void));

    ~HarpCApp();

public:
    HarpCApp() = delete;  // Disable default constructor.
    HarpCApp(HarpCApp& other) = delete; // Disable copy constructor.
    void operator=(const HarpCApp& other) = delete; // Disable assignment operator.

/**
 * \brief initialize the harp core app singleton with parameters and init Tinyusb.
 */
    static HarpCApp& init(uint16_t who_am_i,
                          uint8_t hw_version_major, uint8_t hw_version_minor,
                          uint8_t assembly_version,
                          uint8_t harp_version_major, uint8_t harp_version_minor,
                          uint8_t fw_version_major, uint8_t fw_version_minor,
                          uint16_t serial_number, const char name[],
                          const uint8_t tag[],
                          void* app_reg_values, RegSpecs* app_reg_specs,
                          RegFnPair* reg_fns, size_t app_reg_count,
                          void (* update_fn)(void), void (*reset_fn)(void));

    static inline HarpCApp* self = nullptr; // pointer to the singleton instance.
    static HarpCApp& instance() {return *self;} ///< returns the singleton.

private:
/**
 * \brief entry point for handling incoming harp messages to core registers.
 *  Dispatches message to the appropriate handler.
 */
    void handle_buffered_app_message();

/**
 * \brief update app state. Readable registers can be updated here.
 *  Implements virtual member fn in base class of the same name.
 */
    void update_app_state()
    {update_fn_();}

/**
 * \brief Reset the app state.
 *  Implements virtual member fn in base class of the same name.
 */
    void reset_app()
    {reset_fn_();}

/**
 * \brief send one harp reply read message per app register.
 *  Implements virtual member fn in base class of the same name.
 */
    void dump_app_registers();

/**
 * \brief return app address's specs from the specified register address.
 * \param address is the full address range where 0 is the first core register,
 *  and APP_REG_START_ADDRESS is the first app register.
 * \details used in Harp Core to extract specs for a particular app register.
 */
    const RegSpecs& address_to_app_reg_specs(uint8_t address)
    {return reg_specs_[address - APP_REG_START_ADDRESS];}

// Private Members
    void* reg_values_;
    RegSpecs* reg_specs_;
    RegFnPair* reg_fns_;
    size_t reg_count_;
    void (* update_fn_)(void);
    void (* reset_fn_)(void);
};

#endif //HARP_C_APP_H
