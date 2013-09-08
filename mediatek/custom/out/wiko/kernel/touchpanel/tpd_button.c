#include "tpd.h"
#include <linux/delay.h>

extern struct tpd_device *tpd;
extern int tp_boot_mode;

//#ifdef TPD_HAVE_BUTTON
//static int tpd_keys[TPD_KEY_COUNT] = TPD_KEYS;
//static int tpd_keys_dim[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#define VIRTUAL_KEY_DEB_TIME	3		//Ivan todo: will move to button init later
static unsigned int tpd_keycnt = 0;
static int tpd_keys[TPD_VIRTUAL_KEY_MAX]={0};
static int tpd_keys_dim[TPD_VIRTUAL_KEY_MAX][4];// = {0};
static int tpd_debounce[TPD_VIRTUAL_KEY_MAX];


static ssize_t mtk_virtual_keys_show(struct kobject *kobj,
                   struct kobj_attribute *attr, char *buf) {
    int i, j;
    for(i=0, j=0;i<tpd_keycnt;i++)
        j+=sprintf(buf, "%s%s:%d:%d:%d:%d:%d%s",buf,
           __stringify(EV_KEY),tpd_keys[i],
           tpd_keys_dim[i][0],tpd_keys_dim[i][1],
           tpd_keys_dim[i][2],tpd_keys_dim[i][3],
           (i==tpd_keycnt-1?"\n":":"));
    return j;
}

static struct kobj_attribute mtk_virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.mtk-tpd",
        .mode = S_IRUGO,
    },
    .show = &mtk_virtual_keys_show,
};

static struct attribute *mtk_properties_attrs[] = {
    &mtk_virtual_keys_attr.attr,
    NULL
};

static struct attribute_group mtk_properties_attr_group = {
    .attrs = mtk_properties_attrs,
};

struct kobject *properties_kobj;


void tpd_clr_key_debounce(void)
{
    int i;
    for (i = 0; i < tpd_keycnt; i++)
    {
        //edit by Magnum 2012-2-7

         tpd_debounce[i] = 0; //4//4
         /*
        if(cut_debounce==1)
        {
          tpd_debounce[i] = 4; //4//4
          printk("Magnum tpd_debounce is 4\n");
        } 
        else
        {
		  tpd_debounce[i] = 0;
		  printk("Magnum tpd_debounce is 0\n");
        	}*/
    }
}
 
//[wj add end]
void tpd_button_init(void) {
    int ret = 0, i = 0, j=0;
    tpd->kpd=input_allocate_device();
    /* struct input_dev kpd initialization and registration */
    tpd->kpd->name = TPD_DEVICE "-kpd";
    set_bit(EV_KEY, tpd->kpd->evbit);
    //set_bit(EV_REL, tpd->kpd->evbit);
    //set_bit(EV_ABS, tpd->kpd->evbit);
    for(i=0;i<tpd_keycnt;i++)
        __set_bit(tpd_keys[i], tpd->kpd->keybit);
    tpd->kpd->id.bustype = BUS_HOST;
    tpd->kpd->id.vendor  = 0x0001;
    tpd->kpd->id.product = 0x0001;
    tpd->kpd->id.version = 0x0100;
    if(input_register_device(tpd->kpd))
        TPD_DMESG("input_register_device failed.(kpd)\n");
    set_bit(EV_KEY, tpd->dev->evbit);
    for(i=0;i<tpd_keycnt;i++)
        __set_bit(tpd_keys[i], tpd->dev->keybit);
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if(properties_kobj)
        ret = sysfs_create_group(properties_kobj,&mtk_properties_attr_group);
    if(!properties_kobj || ret)
    printk("failed to create board_properties\n");
}

void tpd_button(unsigned int x, unsigned int y, unsigned int down) {
    int i;
    if(down) {
        for(i=0;i<tpd_keycnt;i++) 
		{
            if(x>=tpd_keys_dim[i][0]-(tpd_keys_dim[i][2]/2) &&
               x<=tpd_keys_dim[i][0]+(tpd_keys_dim[i][2]/2) &&
               y>=tpd_keys_dim[i][1]-(tpd_keys_dim[i][3]/2) &&
               y<=tpd_keys_dim[i][1]+(tpd_keys_dim[i][3]/2) &&
/*               
               !(tpd->btn_state&(1<<i))) {
                input_report_key(tpd->kpd, tpd_keys[i], 1);
                tpd->btn_state|=(1<<i);
                TPD_DEBUG("[mtk-tpd] press key %d (%d)\n",i, tpd_keys[i]);
*/
               !(tpd->btn_state)) {
                //Accept single key only
//Ivan add key debounce
               
                if (tpd_debounce[i] > VIRTUAL_KEY_DEB_TIME)
                {
#ifdef TPD_TYPE_CAPACITIVE		    
                   if(tp_boot_mode==NORMAL_BOOT){
				   
                       tpd_down(x , y, x , y, 1);
                       input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, 0);
                    }
                   else{
				   
                       input_report_key(tpd->kpd, tpd_keys[i], 1);
                    }
#else
                 //  printk("Magnum i am in input key is %d\n",i);
                   input_report_key(tpd->kpd, tpd_keys[i], 1);
#endif
//Ivan FIXME ???
					input_sync(tpd->kpd);
//
                   tpd->btn_state|=(1<<i);
                   TPD_DEBUG("[mt6516-tpd] press key %d (%d)\n",i, tpd_keys[i]);
                }
                else
               {
                  tpd_debounce[i]++;
                   TPD_DEBUG("[mt6516-tpd] debounce key %d (%d)\n",i, tpd_keys[i]);
               }
            }
        }
    } else {
        for(i=0;i<tpd_keycnt;i++) {
            if(tpd->btn_state&(1<<i)) {
#ifdef TPD_TYPE_CAPACITIVE		    		
                if(tp_boot_mode==NORMAL_BOOT){
                    tpd_up(x , y, x , y, NULL);
                    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, 0);
                }
                else{
                     input_report_key(tpd->kpd, tpd_keys[i], 0);
                }
#else
                input_report_key(tpd->kpd, tpd_keys[i], 0);
#endif
                TPD_DEBUG("[mtk-tpd] release key %d (%d)\n",i, tpd_keys[i]);
            }
        }
        tpd_clr_key_debounce();
        tpd->btn_state=0;
    }
}
int tpd_button_discriminate(unsigned int x, unsigned int y) {
    int i;
        for(i=0;i<TPD_KEY_COUNT;i++) {
            if(x>=tpd_keys_dim[i][0]-(tpd_keys_dim[i][2]/2) &&
               x<=tpd_keys_dim[i][0]+(tpd_keys_dim[i][2]/2) &&
               y>=tpd_keys_dim[i][1]-(tpd_keys_dim[i][3]/2) &&
               y<=tpd_keys_dim[i][1]+(tpd_keys_dim[i][3]/2) &&
               !(tpd->btn_state))
               {                       //Accept single key only
                       return tpd_keys[i];
               }
        }
}
void tpd_button_setting(int keycnt, void *keys, void *keys_dim)
{
		tpd_keycnt = keycnt;
		memcpy(tpd_keys, keys, keycnt*4);
		memcpy(tpd_keys_dim, keys_dim, keycnt*4*4);
}

//Ivan added
void tpd_key(unsigned int index, unsigned int down)
{
    if(down) {
        if(!(tpd->btn_state&(1<<(index-1)))) {
            input_report_key(tpd->kpd, tpd_keys[(index-1)], 1);
            tpd->btn_state|=(1<<(index-1));
            TPD_DEBUG("[mt6516-tpd] press key %d (%d)\n",(index-1), tpd_keys[(index-1)]);
        }
    } else {
        if(tpd->btn_state&(1<<(index-1))) {
            input_report_key(tpd->kpd, tpd_keys[(index-1)], 0);
            TPD_DEBUG("[mt6516-tpd] release key %d (%d)\n",(index-1), tpd_keys[(index-1)]);
        }
        tpd->btn_state=0;
    }
}

//#endif
