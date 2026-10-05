/* UART/kernel leaves only; editor, lifecycle and worker come from console.c. */
#include <setjmp.h>
#define USB_EXISTS 1
#define UART_CONSOLE_EXISTS 0
#define CONFIG_CONSOLE_INPUT_MAX_LINE_LEN 128
#define CONFIG_SLIMEVR_USB_DEVICE_MANUFACTURER "test"
#define CONFIG_SLIMEVR_USB_DEVICE_PRODUCT "tracker"
#define FW_STRING "test"
#define FW_GIT_REPO_URL "test"
#define FW_GIT_BRANCH "test"
#define CONSOLE_THREAD_PRIORITY 8
#define BUILD_ASSERT _Static_assert
#define ARG_UNUSED(x) (void)(x)
#define MIN(a,b) ((a)<(b)?(a):(b))
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define K_THREAD_STACK_DEFINE(name,size) unsigned char name[size]
#define K_THREAD_STACK_SIZEOF(name) sizeof(name)
#define DEVICE_DT_GET(node) (&uart_device)
#define DT_CHOSEN(node) 0
struct device { int unused; };
static const struct device uart_device;
struct k_thread { int unused; };
typedef void (*k_thread_entry_t)(void *, void *, void *);
static void k_thread_create(struct k_thread *t, void *s, size_t n, k_thread_entry_t e, void *a, void *b, void *c, int p, int o, int d) {
    (void)t;(void)s;(void)n;(void)e;(void)a;(void)b;(void)c;(void)p;(void)o;(void)d;
}
static jmp_buf worker_idle;
struct k_msgq { unsigned char *data; size_t size, capacity, head, count; };
#define K_MSGQ_DEFINE(name,size,depth,alignment) static unsigned char name##_storage[(size)*(depth)]; static struct k_msgq name={name##_storage,size,depth,0,0}
static int k_msgq_put(struct k_msgq *q,const void *m,int timeout) {
    assert(timeout==K_NO_WAIT); if(q->count==q->capacity)return -ENOMSG;
    size_t slot=(q->head+q->count++)%q->capacity; memcpy(q->data+slot*q->size,m,q->size);return 0;
}
static int k_msgq_get(struct k_msgq *q,void *m,int timeout) {
    if(!q->count && timeout!=K_NO_WAIT)longjmp(worker_idle,1);
    if(!q->count)return -ENOMSG;
    memcpy(m,q->data+q->head*q->size,q->size);q->head=(q->head+1)%q->capacity;q->count--;return 0;
}
struct k_sem { unsigned count, limit; };
#define K_SEM_DEFINE(name,initial,maximum) static struct k_sem name={initial,maximum}
static void k_sem_give(struct k_sem *s) { if(s->count<s->limit)s->count++; }
static int k_sem_take(struct k_sem *s,int timeout) {
    if(s->count){s->count--;return 0;} if(timeout!=K_NO_WAIT)longjmp(worker_idle,1);return -EAGAIN;
}
static unsigned char rx[8192];
static size_t rx_head,rx_count;
static bool rx_enabled,tx_enabled;
static void (*irq_callback)(const struct device *,void *);
static bool device_is_ready(const struct device *d){(void)d;return true;}
static void uart_irq_rx_disable(const struct device *d){(void)d;rx_enabled=false;}
static void uart_irq_tx_disable(const struct device *d){(void)d;tx_enabled=false;}
static void uart_irq_rx_enable(const struct device *d){(void)d;rx_enabled=true;}
static void uart_irq_tx_enable(const struct device *d){(void)d;tx_enabled=true;}
static int uart_irq_callback_user_data_set(const struct device *d,void (*cb)(const struct device *,void *),void *p){(void)d;(void)p;irq_callback=cb;return 0;}
static int uart_poll_in(const struct device *d,unsigned char *b){(void)d;if(!rx_count)return -1;*b=rx[rx_head++];rx_count--;return 0;}
static int uart_irq_update(const struct device *d){(void)d;return 1;}
static int uart_irq_rx_ready(const struct device *d){(void)d;return rx_enabled&&rx_count;}
static int uart_irq_tx_ready(const struct device *d){(void)d;return tx_enabled;}
static int uart_irq_is_pending(const struct device *d){return uart_irq_rx_ready(d)||uart_irq_tx_ready(d);}
static int uart_fifo_read(const struct device *d,uint8_t *b,int n){assert(n==1);return uart_poll_in(d,b)==0?1:0;}
static int uart_fifo_fill(const struct device *d,const uint8_t *b,int n){(void)d;(void)b;return n;}
static void strtolower(char *p){for(;*p;p++)*p=(char)tolower((unsigned char)*p);}
static bool console_feedback_enabled;
static uint32_t console_command_input_generation;
static void console_reset_cancel(void){}
static void console_reject(void){}
static bool console_reset_armed;
static void console_thread(void);
static int console_serial_start(void);
static void console_serial_close(void);
static void console_serial_stop(void);
struct console_cmd { const char *name; void (*fn)(size_t,char **); };
static void backup_dispatch(size_t argc,char **argv){sensor_tcal_backup_command(argc,argv,console_command_input_generation);}
static const struct console_cmd backup_command={"tcal",backup_dispatch};
static const struct console_cmd *console_find_command(const char *name){return strcmp(name,"tcal")==0?&backup_command:NULL;}
static bool console_command_mutates(const struct console_cmd *c,size_t n,char **a){(void)c;(void)n;(void)a;return false;}
#include "console_production.inc"
static void run_worker(void){assert(!in_irq);if(setjmp(worker_idle)==0)console_thread();}
static void receive(const char *text){
    assert(!rx_count);rx_head=0;rx_count=strlen(text);assert(rx_count<=sizeof(rx));memcpy(rx,text,rx_count);
    in_irq=true;irq_callback(console_uart_dev,NULL);in_irq=false;assert(!rx_count);
}
