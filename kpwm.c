/*  Example  kernel module driver 
    by Ludovic Saint-Bauzel (saintbauzel@isir.upmc.fr)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>. 
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <asm/io.h>

#include <asm/uaccess.h>	/* copy_{from,to}_user() */
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/cdev.h>  

#define  DEVICE_NAME "kpwm"    ///< The device will appear at /dev/ebbchar using this value
#define  CLASS_NAME  "kpwm"        ///< The device class -- this is a character device driver
static struct class*  charClass  = NULL; ///< The device-driver class struct pointer
static struct device* charDevice = NULL; ///< The device-driver device struct pointer
int major;
module_param(major,int, S_IRUGO);

#define PERIOD 1200

#define AM33XX_CONTROL_BASE		0x44e10000
// spruh73m.pdf  Issue de L3 memory et L4 memory p179 p181 p182
#define GPIO0_REGISTER 0x44e07000
#define GPIO1_REGISTER 0x4804C000
#define GPIO2_REGISTER 0x481AC000
#define GPIO3_REGISTER 0x481AE000


// p. 183 
#define PWMSS0_REG 0x48300000
#define PWMSS1_REG 0x48302000 // adresse du début du registre PWMSS1 servant à configurer le PWM


#define ECAP_OFF 0x100
#define EQEP_OFF 0x180
#define EPWM_OFF 0x200



// spruh73m.pdf Issue de register description GPIO p 4881 ...
#define GPIO_OE 0x134
#define GPIO_DATAIN 0x138
#define GPIO_DATAOUT 0x13C
#define GPIO_CTRL 0x130
#define GPIO_CLRDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194


// spruh73m.pdf Issue de p 1369
#define OFFSET_PWMSS_CTRL 0x664


#define OFFSET_PIN9_12 0x878
#define GPIO1_28_PIN9_12 28

#define OFFSET_PIN_9_14 0x848
#define GPIO1_18_PIN9_14 18

#define EPWM_TBCTL 0x00
// Bits in TBCTL 
#define CTRMODE 0x0
#define PHSEN 0x2
#define PRDLD 0x3
#define SYNCOSEL 0x4
#define HSPCLKDIV 0x7
#define CLKDIV 0xA 
// Values in TBCTL
#define TB_UP 0x0
#define TB_UPDOWN 0x2 
#define TB_DISABLE 0x0 
#define TB_SHADOW 0x0
#define TB_SYNC_DISABLE 0x3
#define TB_DIV1 0x0
#define TB_DIV2 0x1
#define TB_DIV4 0x2

#define EPWM_CMPCTL 0x0E
// Bits in CMPCTL
#define SHDWAMODE 0x4
#define SHDWBMODE 0x6
#define LOADAMODE 0x0
#define LOADBMODE 0x2
// Values in CMPCTL
#define CC_SHADOW 0x0
#define CC_CTR_ZERO 0x0

#define EPWM_TBCNT 0x08
#define EPWM_TBPRD 0x0A
#define EPWM_TBPHS 0x06

#define EPWM_CMPA 0x12
#define EPWM_CMPB 0x14

#define EPWM_CMPAHR 0x10


#define EPWM_AQCTLA 0x16
// Bits in AQCTL
#define ZRO 0
#define PRD 2
#define CAU 4
#define CAD 6
#define CBU 8
#define CBD 10
// Values
#define AQ_CLEAR 0x1
#define AQ_SET 0x2



//#define ECAP_OFF 0x100
/* ECAP registers and bits definitions */
#define CAP1			0x08
#define CAP2			0x0C
#define CAP3			0x10
#define CAP4			0x14
#define ECCTL2			0x2A
#define ECCTL2_APWM_POL_LOW     (0x1 << 10)
#define ECCTL2_APWM_MODE        (0x1 << 9)
#define ECCTL2_SYNC_SEL_DISA	((0x1 << 7) |(0x1 << 6))
#define ECCTL2_TSCTR_FREERUN	(0x1 << 4)

#define INTC_MIR_CLEAR2 0xC8
#define INTC_MIR_SET2 0xCC
#define EPWM0INT 86 
#define EPWM1INT 87


#define PWM_SET 0

MODULE_DESCRIPTION("Simple ioctl pwm control driver (char)");
MODULE_AUTHOR("TOURLOUR Thomas, MARILLESSE Lucas");
MODULE_LICENSE("GPL");


// CONTROLE MODULE

#define CONTROLE_MODULE_BASE_ADDR 0x44e10000
#define CONTROLE_MODULE_LEN 0x2000
//pwmss ? 

// CLOCK MANAGEMENT

#define CM_PER_BASE_ADDR 0x44e00000
#define CM_PER_LEN 0x4000

#define CM_PER_L4LS_CLKSTCTRL 0
#define CM_PER_L4LS_CLKCTRL 0x60

#define CM_PER_CLKCTRL_LEN 0x04
// POWER RESET MODULE PERIPHERAL REGISTERS

#define PRM_PER_BASE_ADDR 0x44e00c00 

#define PM_PER_PWRSTST 0x8
#define PM_PER_PWRSTCTRL 0xc


static long pwm_ioctl(struct file *file, unsigned int cmd, unsigned long arg) /*fonction utilisée pour modifier manuellement le DUTY*/ 
{
  void __iomem* cm_per_pwm;
  short sregval;
  unsigned short val;
  printk( KERN_DEBUG "char5: ioctl() PWMSET(0x%04x)\n",(unsigned short)arg);
  
  val=(unsigned short)arg;
  if(val > PERIOD)
    val = PERIOD;
  
  switch (cmd) {
  case PWM_SET :
    cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + EPWM_CMPA,4);
    if(!cm_per_pwm) {
      printk("cm_per_pwm_reg ioremap: error\n");
      break;
    }
    else {
      iowrite16(/*A REMPLIR*/(unsigned short)arg,cm_per_pwm); /* le DUTY passé en argument lors de l'appel de la fonction*/
      sregval = ioread16(cm_per_pwm);
      printk("DUTY 0x%04x\n",sregval);
      cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + EPWM_CMPAHR,4); // p. 2325 : Get  Counter Compare HR Register
      //                                                              for output A
      if(!cm_per_pwm) {
	printk("cm_per_pwm_reg ioremap: error\n");
      }
      else {
	sregval = ioread16(cm_per_pwm);
	printk("DUTYHR 0x%04x\n",sregval);
      }
    }
    break;
  default :
    printk(KERN_WARNING "kpwm: 0x%x unsupported ioctl command\n", cmd);
    return -EINVAL;
  }
  return 0;
}

static struct file_operations char_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl =      pwm_ioctl,
};


static void bbb_l4ls_config(void) {
  void __iomem* cm_per_l4ls_clkctrl_reg;
  void __iomem* cm_per_l4ls_clkstctrl_reg;
  int regval = 0;

  if(!request_mem_region(CM_PER_BASE_ADDR + CM_PER_L4LS_CLKCTRL,4,"cm_per_l4ls_clkctrl")) {
    printk(KERN_ERR "request_mem_region: cm_per_l4ls_clkctrl\n");
  }
  else {
    cm_per_l4ls_clkctrl_reg = ioremap(CM_PER_BASE_ADDR + CM_PER_L4LS_CLKCTRL, 4);
    if(!cm_per_l4ls_clkctrl_reg) {
      printk(KERN_ERR "cm_per_l4ls_clkctrl_reg ioremap: error\n");
    }
    else {
      regval = ioread32(cm_per_l4ls_clkctrl_reg);
      printk(KERN_INFO "cm_per_l4ls_clkctrl: %08x\n",regval);
      iowrite32(regval|0x2,cm_per_l4ls_clkctrl_reg); // Enable module
      while((ioread32(cm_per_l4ls_clkctrl_reg)&0x0300)!=0); // Fully functional
      iounmap(cm_per_l4ls_clkctrl_reg);
    }
    release_mem_region(CM_PER_BASE_ADDR + CM_PER_L4LS_CLKCTRL,4);
  }

  if(!request_mem_region(CM_PER_BASE_ADDR+CM_PER_L4LS_CLKSTCTRL,4,"cm_per_l4ls_clksctrl")) {
    printk(KERN_ERR "request_mem_region: cm_per_l4ls_clksctrl\n");
  }
  else {
    cm_per_l4ls_clkstctrl_reg = ioremap(CM_PER_BASE_ADDR+CM_PER_L4LS_CLKSTCTRL,4);
    if(!cm_per_l4ls_clkstctrl_reg) {
      printk(KERN_ERR "cm_per_l4ls_clkctrl_reg ioremap: error\n");
    }
    else {
      printk(KERN_INFO "cm_per_l4ls_clkstctrl: %08x\n",ioread32(cm_per_l4ls_clkstctrl_reg));
      iounmap(cm_per_l4ls_clkstctrl_reg);
    }
    release_mem_region(CM_PER_BASE_ADDR+CM_PER_L4LS_CLKSTCTRL,4);

  }
}

static void bbb_pwmss_config_clk(void) {

  void __iomem* cm_per_pwmss_reg;
  int regval = 0;
  int clkctrl_offset[] = {0xD4,0xCC,0xD8};
  int ii=0;

  for (ii=0;ii<3;ii++){
  if(!request_mem_region(CM_PER_BASE_ADDR + clkctrl_offset[ii],4,"cm_per_pwmssn_clkctrl")) {
    printk(KERN_ERR "request_mem_region: cm_per_pwmss%d_clkctrl\n",ii);
  }
  else {
    cm_per_pwmss_reg = ioremap(CM_PER_BASE_ADDR + clkctrl_offset[ii], CM_PER_CLKCTRL_LEN);
    if(!cm_per_pwmss_reg) {
      printk(KERN_ERR "cm_per_l4ls_clkctrl_reg ioremap: error\n");
    }
    else {
      regval = ioread32(cm_per_pwmss_reg);
      printk(KERN_INFO "cm_per_pwmssx_clkctrl: %08x\n",regval);
      iowrite32(regval|0x2,cm_per_pwmss_reg); // Enable module
      while((ioread32(cm_per_pwmss_reg)&0x0300)!=0); // Fully functional
      iounmap(cm_per_pwmss_reg);
    }
    release_mem_region(CM_PER_BASE_ADDR + clkctrl_offset[ii],4);
  }}
  
}

static int  pinmuxPWM(void)
{
  int ret=0;
  int regval ;                 
  void __iomem* cm_per_gpio;
  
  /********************* PINMUX spruh73m.pdf p. 179 - 1370 - 1426 ****************/
  cm_per_gpio = ioremap(AM33XX_CONTROL_BASE+OFFSET_PIN_9_14,4);
  if(!cm_per_gpio) {
    printk("cm_per_gpio_reg ioremap: error\n");
  }
  else {
    regval = ioread32(cm_per_gpio) ; //get PWM GPIO MUX register value
    iowrite32(0x00,cm_per_gpio); //reset PWM GPIO MUX
    regval = ioread32(cm_per_gpio) ; 
    iowrite32(/*A REMPLIR*/0X00000006,cm_per_gpio);// p. 1426 : GPIO MUX to PWM (Mode 6)
    //                                                  Pulldown (sel. and enabled)
    regval = ioread32(cm_per_gpio) ; //enabled? PWM GPIO MUX
    printk("GPX1CON register mux (1e) : 0x%08x\n", regval);
    iounmap(cm_per_gpio);
  }
  return ret;
  
}
static int initPWM(void)
{

  int regval ;                 
  short sregval;
  void __iomem* cm_per_pwm;

  /******* Enable L4LS and Clocks *******/
  bbb_l4ls_config();
  bbb_pwmss_config_clk();
  /********************* PWM enabling spruh73m.pdf p. 1369 - 1426 ****************/

  
  // P. 1369
  cm_per_pwm = ioremap(AM33XX_CONTROL_BASE+OFFSET_PWMSS_CTRL,4);
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    iowrite32(0x7,cm_per_pwm);
    
    regval = ioread32(cm_per_pwm) ; // p. 1407 : enable PWMSS1 0x2 / pwmss0 0x1 /pwmss2 0x4 tbclken
    printk("PWMSSx tbclkenables if Ox7 : 0x%08x\n",regval);
    
  }

  /*Enable PWMSS CLKCONFIG */
  cm_per_pwm = ioremap(PWMSS1_REG + 0x08,4);
  if(!cm_per_pwm) {
    printk("PWMSS1 CLKCONFIG ioremap: error\n");
  }
  else {
    regval = ioread32(cm_per_pwm) ;  // p. 2235 : reset / enable PWMSS1 clk
    printk("PWMSSx clken if Ox111 : 0x%08x\n",regval); 
    
  }
  cm_per_pwm = ioremap(PWMSS1_REG + 0x0c,4);
  if(!cm_per_pwm) {
    printk("PWMSS1 CLKSTATUS ioremap: error\n");
  }
  else {
    regval = ioread32(cm_per_pwm) ; 
    printk("PWMSSx clkSTATUS  : 0x%08x\n",regval);
  }
  
  cm_per_pwm = ioremap(PWMSS1_REG + 0x04,4); // p. 2232 : see EPWMSS SYSCFG
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    sregval = ioread32(cm_per_pwm);
    printk("EPWMSS SYSCFG 0x%08x\n",sregval);
    
  }
  
  cm_per_pwm = ioremap(PWMSS1_REG + 0x08,4); // p. 2326 : see EPWMSS CLKCFG
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    sregval = ioread32(cm_per_pwm);
    printk("EPWMSS CLKCFG 0x%08x\n",sregval);
  }   
  
  cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + 0x0,2); // p. 2326 : see TBCTL
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    iowrite16(/*A REMPLIR*/0x0080,cm_per_pwm); // Very Important This register start in freezed mode.
    //                             Define it as a count up instead blocked (0x83) 
    sregval = ioread16(cm_per_pwm);
    printk("TBCTL 0x%04x\n",sregval);
  }
  
  cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + 0x02,2); // p. 2326 : see TBCTLSTS
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    sregval = ioread16(cm_per_pwm);
    printk("TBSTS 0x%04x\n",sregval);
  }
  
  /***************** PWM Configuration based on scenario p.2305 ************************/ 
  
  cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + EPWM_AQCTLA,4); // p. 2325 : Get  Action Qualifier Register
  //                                                                         for output A    
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    sregval = ioread16(cm_per_pwm) ; //reset AQ Actions
    iowrite16(0,cm_per_pwm);
    sregval = ioread16(cm_per_pwm) ; //enable AQ Actions
    iowrite16(/*A REMPLIR*/(1<<2|1<<5),cm_per_pwm); /* quand le compteur est égal à la periode on fait un clear, quand il est égal à la valeur du registre CMPA (DUTY) on fait un set*/
    sregval = ioread16(cm_per_pwm) ; //read AQ Actions
    printk("AQCTLA 0x%04x\n",sregval);
  }
	
  cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + 0x1E,4); // p. 2325 : DeadBand Control Register
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    sregval = ioread16(cm_per_pwm) ; //reset DeadBand
    iowrite16(0,cm_per_pwm);
    sregval = ioread16(cm_per_pwm);
    printk("DBCTL 0x%04x\n",sregval);
  }

  cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + EPWM_CMPA,4); // p. 2325 : Get  Counter Compare  Register
  //                                                              for output A
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    iowrite16(/*A REMPLIR*/0X02bc,cm_per_pwm);  /*on initialise le DUTY à 700ns*/
    sregval = ioread16(cm_per_pwm);
    printk("DUTY 0x%04x\n",sregval);
  }
  cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + EPWM_CMPAHR,4); // p. 2325 : Get  Counter Compare HR Register
  //                                                              for output A
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    iowrite16(0x00,cm_per_pwm); //Reset 0x100
    sregval = ioread16(cm_per_pwm);
    printk("DUTYHR 0x%04x\n",sregval);
  }
	
  cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + EPWM_TBPRD,4);  // p. 2325 : Get  Counter Compare  Register
  //                                                              for output A
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    iowrite16(/*A REMPLIR*/0X04b0,cm_per_pwm); /*on initialise la période à 1200ns, cette valeur sera invariante*/
    sregval = ioread16(cm_per_pwm);
    printk("PERIOD 0x%04x\n",sregval);
  }

  return 0;
}

// this gets called on module init
static int __init kernmodex_init(void)
{
  int ret;
  
  printk(KERN_INFO "Loading example driver by Ludo...\n");
  ret = pinmuxPWM();
  if (ret < 0)
    {
      printk("problem in pinmux of PWM");
      return ret;
    }

  ret= initPWM();
  if (ret < 0)
    {
      printk("problem in pinmux of PWM");
      return ret;
    }
  /************** Char Device creation *****************************/
  ret = register_chrdev(major, CLASS_NAME, &char_fops);
  if (ret < 0) {
    printk(KERN_WARNING "KPWM: unable to get a major\n");

    return ret;
  }
  if (major == 0)
    major = ret; /* dynamic value */
  
  printk(KERN_INFO "KPWM: successfully loaded with major %d\n", major);
  
  charClass = class_create(THIS_MODULE, CLASS_NAME);
  if (IS_ERR(charClass)){                // Check for error and clean up if there is
    unregister_chrdev(major, DEVICE_NAME);
    printk(KERN_ALERT "Failed to register device class\n");
    return PTR_ERR(charClass);          // Correct way to return an error on a pointer
  }
  printk(KERN_INFO "KPWM device class registered correctly\n");
  
  
  // Register the device driver
  
  
  charDevice = device_create(charClass, NULL, MKDEV(major, 0), NULL, DEVICE_NAME );
  if (IS_ERR(charDevice)){               // Clean up if there is an error
    class_destroy(charClass);           // Repeated code but the alternative is goto statements
    unregister_chrdev(major, DEVICE_NAME);
    printk(KERN_ALERT "Failed to create the device\n");
    return PTR_ERR(charDevice);
  }
  printk(KERN_INFO "KPWM: device class created correctly\n"); // Made it! device was initialized
  
  return 0;
  
}

// this gets called when module is getting unloaded
static void __exit kernmodex_exit(void)
{
  
  void __iomem* cm_per_pwm;
  short sregval;
  
  device_destroy(charClass, MKDEV(major, 0));

  class_unregister(charClass);  
  class_destroy(charClass);
  unregister_chrdev(major, CLASS_NAME);


  cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + EPWM_CMPA,4);
  if(!cm_per_pwm) {
    printk("cm_per_pwm_reg ioremap: error\n");
  }
  else {
    iowrite16(0x00,cm_per_pwm);
    sregval = ioread16(cm_per_pwm);
    printk("DUTY 0x%04x\n",sregval);
  }
  
  /* cm_per_pwm = ioremap(PWMSS1_REG + EPWM_OFF + 0x0,2); // p. 2326 : see TBCTL */
  /* if(!cm_per_pwm) { */
  /*   printk("cm_per_pwm_reg ioremap: error\n"); */
  /* } */
  /* else { */
  /*   iowrite16(0x83,cm_per_pwm); // Very Important This register start in freezed mode. */
  /*   //                             Define it as a count up instead blocked (0x83)  */
  /*   sregval = ioread16(cm_per_pwm); */
  /*   printk("TBCTL 0x%04x\n",sregval); */
  /* } */

    

  printk(KERN_INFO "Example driver by Ludo removed.\n");
  
  
}

// setting which function to call on module init and exit
module_init(kernmodex_init);
module_exit(kernmodex_exit);


MODULE_VERSION("0.3");
