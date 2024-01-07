#include "shared.h"

const uint8 txt_lookup[256][2] = {
{0000, 0000}, {0x01, 0x01}, {0x02, 0x02}, {0x03, 0x03}, {0x04, 0x04}, {0x05, 0x05}, {0x06, 0x06}, {0x07, 0x07}, {0x08, 0x08}, {0x09, 0x09}, {0x0a, 0x0a}, {0x0b, 0x0b}, {0x0c, 0x0c}, {0x0d, 0x0d}, {0x0e, 0x0e}, {0x0f, 0x0f}, {0000, 0x01}, {0x01, 0x01}, {0x02, 0x01}, {0x03, 0x01}, {0x04, 0x01}, {0x05, 0x01}, {0x06, 0x01}, {0x07, 0x01}, {0x08, 0x01}, {0x09, 0x01}, {0x0a, 0x01}, {0x0b, 0x01}, {0x0c, 0x01}, {0x0d, 0x01}, {0x0e, 0x01}, {0x0f, 0x01}, {0000, 0x02}, {0x01, 0x02}, {0x02, 0x02}, {0x03, 0x02}, {0x04, 0x02}, {0x05, 0x02}, {0x06, 0x02}, {0x07, 0x02}, {0x08, 0x02}, {0x09, 0x02}, {0x0a, 0x02}, {0x0b, 0x02}, {0x0c, 0x02}, {0x0d, 0x02}, {0x0e, 0x02}, {0x0f, 0x02}, {0000, 0x03}, {0x01, 0x03}, {0x02, 0x03}, {0x03, 0x03}, {0x04, 0x03}, {0x05, 0x03}, {0x06, 0x03}, {0x07, 0x03}, {0x08, 0x03}, {0x09, 0x03}, {0x0a, 0x03}, {0x0b, 0x03}, {0x0c, 0x03}, {0x0d, 0x03}, {0x0e, 0x03}, {0x0f, 0x03}, {0000, 0x04}, {0x01, 0x04}, {0x02, 0x04}, {0x03, 0x04}, {0x04, 0x04}, {0x05, 0x04}, {0x06, 0x04}, {0x07, 0x04}, {0x08, 0x04}, {0x09, 0x04}, {0x0a, 0x04}, {0x0b, 0x04}, {0x0c, 0x04}, {0x0d, 0x04}, {0x0e, 0x04}, {0x0f, 0x04}, {0000, 0x05}, {0x01, 0x05}, {0x02, 0x05}, {0x03, 0x05}, {0x04, 0x05}, {0x05, 0x05}, {0x06, 0x05}, {0x07, 0x05}, {0x08, 0x05}, {0x09, 0x05}, {0x0a, 0x05}, {0x0b, 0x05}, {0x0c, 0x05}, {0x0d, 0x05}, {0x0e, 0x05}, {0x0f, 0x05}, {0000, 0x06}, {0x01, 0x06}, {0x02, 0x06}, {0x03, 0x06}, {0x04, 0x06}, {0x05, 0x06}, {0x06, 0x06}, {0x07, 0x06}, {0x08, 0x06}, {0x09, 0x06}, {0x0a, 0x06}, {0x0b, 0x06}, {0x0c, 0x06}, {0x0d, 0x06}, {0x0e, 0x06}, {0x0f, 0x06}, {0000, 0x07}, {0x01, 0x07}, {0x02, 0x07}, {0x03, 0x07}, {0x04, 0x07}, {0x05, 0x07}, {0x06, 0x07}, {0x07, 0x07}, {0x08, 0x07}, {0x09, 0x07}, {0x0a, 0x07}, {0x0b, 0x07}, {0x0c, 0x07}, {0x0d, 0x07}, {0x0e, 0x07}, {0x0f, 0x07}, {0000, 0x08}, {0x01, 0x08}, {0x02, 0x08}, {0x03, 0x08}, {0x04, 0x08}, {0x05, 0x08}, {0x06, 0x08}, {0x07, 0x08}, {0x08, 0x08}, {0x09, 0x08}, {0x0a, 0x08}, {0x0b, 0x08}, {0x0c, 0x08}, {0x0d, 0x08}, {0x0e, 0x08}, {0x0f, 0x08}, {0000, 0x09}, {0x01, 0x09}, {0x02, 0x09}, {0x03, 0x09}, {0x04, 0x09}, {0x05, 0x09}, {0x06, 0x09}, {0x07, 0x09}, {0x08, 0x09}, {0x09, 0x09}, {0x0a, 0x09}, {0x0b, 0x09}, {0x0c, 0x09}, {0x0d, 0x09}, {0x0e, 0x09}, {0x0f, 0x09}, {0000, 0x0a}, {0x01, 0x0a}, {0x02, 0x0a}, {0x03, 0x0a}, {0x04, 0x0a}, {0x05, 0x0a}, {0x06, 0x0a}, {0x07, 0x0a}, {0x08, 0x0a}, {0x09, 0x0a}, {0x0a, 0x0a}, {0x0b, 0x0a}, {0x0c, 0x0a}, {0x0d, 0x0a}, {0x0e, 0x0a}, {0x0f, 0x0a}, {0000, 0x0b}, {0x01, 0x0b}, {0x02, 0x0b}, {0x03, 0x0b}, {0x04, 0x0b}, {0x05, 0x0b}, {0x06, 0x0b}, {0x07, 0x0b}, {0x08, 0x0b}, {0x09, 0x0b}, {0x0a, 0x0b}, {0x0b, 0x0b}, {0x0c, 0x0b}, {0x0d, 0x0b}, {0x0e, 0x0b}, {0x0f, 0x0b}, {0000, 0x0c}, {0x01, 0x0c}, {0x02, 0x0c}, {0x03, 0x0c}, {0x04, 0x0c}, {0x05, 0x0c}, {0x06, 0x0c}, {0x07, 0x0c}, {0x08, 0x0c}, {0x09, 0x0c}, {0x0a, 0x0c}, {0x0b, 0x0c}, {0x0c, 0x0c}, {0x0d, 0x0c}, {0x0e, 0x0c}, {0x0f, 0x0c}, {0000, 0x0d}, {0x01, 0x0d}, {0x02, 0x0d}, {0x03, 0x0d}, {0x04, 0x0d}, {0x05, 0x0d}, {0x06, 0x0d}, {0x07, 0x0d}, {0x08, 0x0d}, {0x09, 0x0d}, {0x0a, 0x0d}, {0x0b, 0x0d}, {0x0c, 0x0d}, {0x0d, 0x0d}, {0x0e, 0x0d}, {0x0f, 0x0d}, {0000, 0x0e}, {0x01, 0x0e}, {0x02, 0x0e}, {0x03, 0x0e}, {0x04, 0x0e}, {0x05, 0x0e}, {0x06, 0x0e}, {0x07, 0x0e}, {0x08, 0x0e}, {0x09, 0x0e}, {0x0a, 0x0e}, {0x0b, 0x0e}, {0x0c, 0x0e}, {0x0d, 0x0e}, {0x0e, 0x0e}, {0x0f, 0x0e}, {0000, 0x0f}, {0x01, 0x0f}, {0x02, 0x0f}, {0x03, 0x0f}, {0x04, 0x0f}, {0x05, 0x0f}, {0x06, 0x0f}, {0x07, 0x0f}, {0x08, 0x0f}, {0x09, 0x0f}, {0x0a, 0x0f}, {0x0b, 0x0f}, {0x0c, 0x0f}, {0x0d, 0x0f}, {0x0e, 0x0f}, {0x0f, 0x0f} };
