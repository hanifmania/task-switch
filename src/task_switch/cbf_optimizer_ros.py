#!/usr/bin/env python
# -*- coding: utf-8 -*-

from task_switch.cbf_qp_optimizer import CBFOptimizer

class CBFOptimizerROS(CBFOptimizer):

    # override
    def __init__(self):
        super(CBFOptimizerROS,self).__init__()
        
        # cbf activations, these will be overwritten by dycon 
        self.activate_cbf = True
        self.activate_fieldcbf = False
        self.activate_chargecbf = False
        self.activate_pcccbf = True
        self.activate_staycbf = False
        self.activate_collisioncbf = True
        # if any cbf are not activated, set active_cbf False
        if not any([self.activate_fieldcbf,self.activate_chargecbf,self.activate_pcccbf,self.activate_collisioncbf]):
            self.activate_cbf = False 

        
        # cbf slack weight, these will be overwritten by dycon 
        self.fieldcbf_slack_weight = 1.0
        self.chargecbf_slack_weight = 1.0
        self.pcccbf_slack_weight = 1.0
        self.staycbf_slack_weight = 1.0

        # input range constraint, these will be overwritten by dycon
        self.activate_umax = True
        self.umax = 2.0
        self.umin = -self.umax 




    def updateCbfConfig(self,config):
        # update cbf activations status
        self.activate_cbf = config.activate_cbf
        self.activate_fieldcbf = config.activate_fieldcbf
        self.activate_chargecbf = config.activate_chargecbf
        self.activate_pcccbf = config.activate_pcccbf
        self.activate_staycbf = config.activate_staycbf
        self.activate_collisioncbf = config.activate_collisioncbf
        # if any cbf are not activated, set active_cbf False
        if not any([self.activate_fieldcbf,self.activate_chargecbf,self.activate_pcccbf,self.activate_collisioncbf,self.activate_staycbf]):
            self.activate_cbf = False 

        # update cbf slack weight
        self.fieldcbf_slack_weight = config.fieldcbf_slack_weight
        self.chargecbf_slack_weight = config.chargecbf_slack_weight
        self.pcccbf_slack_weight = config.pcccbf_slack_weight
        self.staycbf_slack_weight = config.staycbf_slack_weight

        # update input range constraint
        self.updateInputRange(config.activate_umax,config.umax,-config.umax)