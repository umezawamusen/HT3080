void ht3080_write_char(int com, char c);

void ht3080_main(void);
void ht3080_end_thread(void);
int ht3080_str_avail(int com);
void set_rs485_mode(int mode);

#define MAX_TTY 8
