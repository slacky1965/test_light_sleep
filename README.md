# test_light_sleep

Comment // in user_main.c 198 line

    i2c_master_gpio_init();

and test light sleep - worked

Uncomment in user_main.c 198 line (initialize i2c GPIO0 and GPIO2)

    i2c_master_gpio_init();
    
and test light sleep - not worked!

I do not know how to do.
