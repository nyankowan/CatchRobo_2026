#include "stdbool.h"

typedef enum{
    MICON_TYPE_ROBOMAS_CONTROLLER,
    MICON_TYPE_UPPER_ARM,
    MICON_TYPE_LOWER_ARM,
}micon_type_t;

void micon_connection_init();
bool get_connection(micon_type_t m);
void micon_connection_update();
void micon_connection_dump();