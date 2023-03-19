.. _xenvm_dom0:

Xen VM Dom0: universal shield for XEN VM Dom-0
##############################################

Overview
********

This virtual shield allows user to build Zephyr as a Xen initial domain (Dom0). The feature
is implemented as shield to allow support for any compatible platform.

How to add support of a new board
*********************************

* add mapping of hypervisor node to appropriate SoC;
* add board dts overlay to this shield which deletes/adds memory and deletes UART nodes.

Programming
***********

Correct shield designation for Xen VM must
be entered when you invoke ``west build``.
For example:

.. zephyr-app-commands::
   :zephyr-app: samples/synchronization
   :board: rcar_h3ulcb_ca57
   :shield: xenvm_dom0
   :goals: build
