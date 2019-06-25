/*copyright (C) 2012 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef _UEI1700_H_
#define _UEI1700_H_

#include <linux/notifier.h>
#include <linux/fb.h>

struct ir_remocon_data {
	struct mutex	mutex;
	int uei1700_power_supply;
	struct kobject *uei1700_obj;
	struct notifier_block	fb_notif;
};
#endif /* _IR_REMOTE_UEI1700_H_ */

