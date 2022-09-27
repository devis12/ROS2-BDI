;; domain file: recycling-agent-domain.pddl

(define (domain recycling-agent-domain)

    (:requirements :strips :typing :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        recycling_agent
        plastic_bin paper_bin - bin
        plastic paper - litter
        cell
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates
        (should_patrol ?c - cell)
        (in ?a - recycling_agent ?c - cell)
        (bin_pose ?b - bin ?c - cell)
        (near ?c1 - cell ?c2 - cell)
        (litter_pose ?l - litter ?c - cell)
        (free ?c - cell)
        (holding ?a - recycling_agent ?l - litter )
        (recycled ?l - litter)
    )

    (:functions
        (loaded_amount ?a - recycling_agent)
        (detection_depth ?a - recycling_agent)
    )

    (:durative-action litter_pickup
        :parameters (?a - recycling_agent ?l - litter ?c - cell)
        :duration (= ?duration 3)
        :condition (and
            (at start (< (loaded_amount ?a) 3))            
            (at start (litter_pose ?l ?c))
            (over all (in ?a ?c))        
        )
        :effect (and
            (at start (not(litter_pose ?l ?c)))
            (at end (increase (loaded_amount ?a) 1))
            (at end (holding ?a ?l))
        )
    )

    (:durative-action move
        :parameters (?a - recycling_agent ?c1 ?c2 - cell)
        :duration (= ?duration 1)
        :condition (and           
            (at start (in ?a ?c1))     
            (over all (free ?c2))  
            (over all (near ?c1 ?c2))            
        )
        :effect (and
            (at start (not(in ?a ?c1)))
            (at start (free ?c1))
            (at end (in ?a ?c2))
            (at end (not(free ?c2)))
        )
    )

    (:durative-action recycle_paper
        :parameters (?a - recycling_agent ?pl - paper ?pb - paper_bin ?c ?cpl - cell)
        :duration (= ?duration 3)
        :condition (and           
            (at start (holding ?a ?pl))                
            (over all (in ?a ?c)) 
            (over all (bin_pose ?pb ?cpl)) 
            (over all (near ?c ?cpl)) 
        )
        :effect (and
            (at start (not(holding ?a ?pl)))
            (at start (decrease (loaded_amount ?a) 1))
            (at end (recycled ?pl))
        )
    )

    (:durative-action recycle_plastic
        :parameters (?a - recycling_agent ?pl - plastic ?pb - plastic_bin ?c ?cpl - cell)
        :duration (= ?duration 3)
        :condition (and           
            (at start (holding ?a ?pl))                
            (over all (in ?a ?c)) 
            (over all (bin_pose ?pb ?cpl)) 
            (over all (near ?c ?cpl)) 
        )
        :effect (and
            (at start (not(holding ?a ?pl)))
            (at start (decrease (loaded_amount ?a) 1))
            (at end (recycled ?pl))
        )
    )
)
