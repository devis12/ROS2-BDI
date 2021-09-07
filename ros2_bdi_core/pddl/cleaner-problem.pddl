``` 
(define (problem moving-agent-problem)
    
    (:domain moving-agent-domain)

    (:objects 
        mov_ag1 - robot
		bathroom - waypoint
        dock - waypoint
    )

	(:init 
        (in mov_ag1 dock)
        (recharging_station dock)
		(= (battery_charge mov_ag1) 30)
    )

    (:goal 
        (and (in mov_ag1 bathroom))
    )
)
``` 