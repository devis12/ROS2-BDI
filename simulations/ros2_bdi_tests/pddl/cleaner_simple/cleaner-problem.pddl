( define ( problem problem_1 ) 
    ( :domain cleaner-domain ) 
    ( :objects 
        dock kitchen - waypoint
        cleaner - robot
    ) 
    ( :init 
        (recharging_station dock)
        (workfree cleaner)
        (in cleaner dock)
        (=(battery_charge cleaner) 90)
    ) 
    ( :goal 
        ( and 
            ( cleaned kitchen ) 
        ) 
    ) 
)