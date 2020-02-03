#lang racket
;(require rosetta/autocad) (define I_MT_STEEL 1) (define analysis-nodes-height (make-parameter 1)) (define-syntax-rule (with-simulation . args) #t) (define add-radiance-polygon! list) (define-syntax-rule (add-radiance-shape! x) #t)
(require rosetta/robot/backend)(define load-beam-family list) (define create-layer list) (define-syntax-rule (with-current-layer l body ...) (begin body ...)) (define analysis-nodes-height (make-parameter 1)) (define-syntax-rule (with-simulation . args) #t) (define add-radiance-polygon! list) (define-syntax-rule (add-radiance-shape! x) #t) (define panel list) (define loft list) (define polygon list)

(delete-all-shapes)

(define cmd-vector (current-command-line-arguments))
(define angle-0 (string->number (vector-ref cmd-vector 0)))
(define angle-1 (string->number (vector-ref cmd-vector 1)))
(define angle-2 (string->number (vector-ref cmd-vector 2)))
(define pos-0 (string->number (vector-ref cmd-vector 3)))
(define pos-1 (string->number (vector-ref cmd-vector 4)))
(define pos-2 (string->number (vector-ref cmd-vector 5)))

;PROPRIEDADES E FAMÍLIAS

(define fixed-xyz-truss-node-family (truss-node-family-element (default-truss-node-family)
                                                               ;#:radius 0.1
                                                               #:support (create-node-support "SupportA" #:ux #t #:uy #t #:uz #t)))

(define fixed-z-truss-node-family (truss-node-family-element (default-truss-node-family)
                                                             ;#:radius 0.1
                                                             #:support (create-node-support "SupportB" #:ux #f #:uy #f #:uz #t)))

(default-truss-bar-family (truss-bar-family-element (default-truss-bar-family)
                                                    ;#:radius 0.05
                                                    #:material (list "ElasticIsotropic"
                                                                     I_MT_STEEL
                                                                     "Steel"
                                                                     "I'm really steel"
                                                                     210000000000.0
                                                                     0.3
                                                                     81000000000.0
                                                                     77010.0
                                                                     1.2E-05
                                                                     0.04
                                                                     235000000.0
                                                                     360000000.0)
                                                    #:section (list "Tube"
                                                                    "ElasticIsotropic"
                                                                    #f
                                                                    (list (list #t ;;solid?
                                                                                0.1
                                                                                0.01)))))
(define truss-node-family (truss-node-family-element (default-truss-node-family)
                                                               #:radius 0.1
                                                               ))


(define (no-trelica p)
  (truss-node p))
  ;(truss-node p truss-node-family))

(define (fixed-xyz-no-trelica p)
  (truss-node p fixed-xyz-truss-node-family))

(define (fixed-z-no-trelica p)
  (truss-node p fixed-z-truss-node-family))

(define (barra-trelica p0 p1)
  (truss-bar p0 p1))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;LAYERS

(define node-layer (create-layer "Nodes"))
(define fixed-node-layer (create-layer "Fixed Nodes"))
(define bar-layer (create-layer "Bars"))
(define radiation-layer (create-layer "Radiation"))

;TRELIÇA

(define (nos-trelica ps)
  (if (null? ps)
    #t
    (begin
      (no-trelica (car ps))
      (nos-trelica (cdr ps)))))

(define (fixed-nos-trelica ps)
  (fixed-xyz-no-trelica (car ps))
  (nos-trelica (drop-right (cdr ps) 1))
  (fixed-xyz-no-trelica (last ps)))

(define (barras-trelica ps qs)
  (if (or (null? ps) (null? qs))
    #t
    (begin
      (barra-trelica (car ps) (car qs))
      (barras-trelica (cdr ps) (cdr qs)))))

(define intermediate-loc intermediate-point)

(define (trelica-espacial curvas f (first? #t))
  (let ((as (car curvas))
        (bs (cadr curvas))
        (cs (caddr curvas)))
    (if first?
        (fixed-nos-trelica as)
        (nos-trelica as))
    (nos-trelica bs)
    (if (null? (cdddr curvas))
        (begin
          (fixed-nos-trelica cs)
          (barras-trelica (cdr cs) cs))
        (begin
          (trelica-espacial (cddr curvas) f #f)
          (barras-trelica bs (cadddr curvas))))
    (barras-trelica as cs)
    (barras-trelica bs as)
    (barras-trelica bs cs)
    (barras-trelica bs (cdr as))
    (barras-trelica bs (cdr cs))
    (barras-trelica (cdr as) as)
    (barras-trelica (cdr bs) bs)))

;TRELIÇA

(define attractors (make-parameter (list (xyz 5 5 5))))

(define (affect-radius r p)
  (* r (+ 1 (* +0.5 (expt 1.4
                          (- (apply min (map (lambda (attractor) (distance p attractor))
                                             (attractors)))))))))

(define (pontos-arco p r fi psi0 psi1 dpsi)
  (if (> psi0 psi1)
    (list)
    (cons (+sph p (affect-radius r (+sph p r fi psi0)) fi psi0)
          (pontos-arco p r fi (+ psi0 dpsi) psi1 dpsi))))

(define (coordenadas-trelica-ondulada p rac rb l fi psi0 psi1 dpsi
                                     alfa0 alfa1 d-alfa d-r)
  (if (>= alfa0 alfa1)
    (list (pontos-arco
            (+pol p (/ l 2.0) (- fi pi/2))
            (+ rac (* d-r (sin alfa0)))
            fi psi0 psi1 dpsi))
    (cons
     (pontos-arco
       (+pol p (/ l 2.0) (- fi pi/2))
       (+ rac (* d-r (sin alfa0)))
       fi psi0 psi1 dpsi)
     (cons
      (pontos-arco
        p
        (+ rb (* d-r (sin alfa0)))
        fi (+ psi0 (/ dpsi 2)) (- psi1 (/ dpsi 2)) dpsi)
      (coordenadas-trelica-ondulada
        (+pol p l (+ fi pi/2))
        rac rb l fi psi0 psi1 dpsi
        (+ alfa0 d-alfa) alfa1 d-alfa d-r)))))

(define (trelica-ondulada p rac rb l n fi psi0 psi1
                          alfa0 alfa1 d-alfa d-r)
  (trelica-espacial
   (coordenadas-trelica-ondulada
    p rac rb l fi psi0 psi1 (/ (- psi1 psi0) n)
    alfa0 alfa1 d-alfa d-r)
   panel))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

#;(parameterize ((attractors (list (+y (sph 10 0 angle-0) pos-0)
                                   (+y (sph 10 0 angle-1) pos-1)
                                   (+y (sph 10 0 angle-2) pos-2))))
    (trelica-ondulada (xyz 0 0 0) 10 9 1.0 10 0 -pi/2 pi/2 0 4pi (/ pi 8) 0.1))

;ANÁLISE ROBOT

(require (prefix-in % rosetta/robot/robot-com))

#;#;#;#;
(require (prefix-in ac: rosetta/autocad))
(ac:delete-all-shapes)
(define red (rgb 255 0 0))
(with-robot-analysis (results)
  (trelica-ondulada (xyz 0 0 0) 10 9 1.0 10 0 -pi/2 pi/2 0 4pi (/ pi 8) 1)
  (vz -50000)
  (let* ((node-radius 0.04)
         (bar-radius 0.02)
         (factor 100)
         (node-displacement (lambda (node)
                              (let ((node-id (%truss-node-data-id node)))
                                (let ((d (%node-displacement (%Displacements (%nodes results)) node-id 1)))
                                  (v*r (vxyz (%UX d) (%UY d) (%UZ d)) factor))))))
    (for ((node (in-hash-values (%added-nodes))))
      (let ((node-id (%truss-node-data-id node)))
        (let ((d (node-displacement node)))
          (let ((p (%truss-node-data-loc node)))
            (ac:sphere p node-radius)
            (ac:shape-color (ac:sphere (p+v p d) node-radius) red)))))
    (for ((bar (in-hash-values (%added-bars))))
      (let ((node0 (%truss-bar-data-node0 bar))
            (node1 (%truss-bar-data-node1 bar)))
        (let ((p0 (%truss-node-data-loc node0))
              (p1 (%truss-node-data-loc node1)))
          (ac:cylinder p0 bar-radius p1)
          (let ((d0 (node-displacement node0))
                (d1 (node-displacement node1)))
            (ac:shape-color (ac:cylinder (p+v p0 d0) bar-radius (p+v p1 d1)) red)))))))


#;
(with-robot-analysis (results)
  (trelica-ondulada (xyz 0 0 0) 10 9 1.0 #;20 10 0 -pi/2 pi/2 0 4pi (/ pi 4) #;(/ pi 8) 1)
  (vz -50000)
  (let* ((node-radius 0.08)
         (bar-radius 0.04)
         (factor 100))
    (for ((node (in-hash-values (%added-nodes))))
      (let ((node-id (%truss-node-data-id node)))
        (let ((p (%truss-node-data-loc node)))
          (ac:sphere p node-radius))))
    (let* ((bars (hash-values (%added-bars)))
           (bars-id (map %truss-bar-data-id bars))
           (bars-stress (map (lambda (bar-id)
                               (abs (%bar-max-stress results bar-id 1)))
                             bars-id)))
      (let ((max-stress (argmax identity bars-stress))
            (min-stress (argmin identity bars-stress)))
        (for ((bar (in-list bars))
              (bar-in (in-list bars-id))
              (bar-stress (in-list bars-stress)))
          (let ((node0 (%truss-bar-data-node0 bar))
                (node1 (%truss-bar-data-node1 bar)))
            (let ((p0 (%truss-node-data-loc node0))
                  (p1 (%truss-node-data-loc node1)))
              (ac:shape-color (ac:cylinder p0 bar-radius p1)
                              (color-in-range bar-stress min-stress max-stress)))))))))
(display
(with-robot-analysis (results)
  (parameterize ((attractors (list (+y (sph 10 0 angle-0) pos-0)
                                   (+y (sph 10 0 angle-1) pos-1)
                                   (+y (sph 10 0 angle-2) pos-2))))
    (trelica-ondulada (xyz 0 0 0) 10 9 1.0 10 0 -pi/2 pi/2 0 4pi (/ pi 8) 0.1))
  (vz -50000)
  (let* ((node-radius 0.04)
         (bar-radius 0.02)
         (factor 100))
    (apply min
              (for/list ((node (in-hash-values (%added-nodes))))
                (cz (v*r (%node-displacement-vector
                          results
                          (%truss-node-data-id node)
                          1)
                         factor))))))
)
;(trelica-ondulada p rac rb l n fi psi0 psi1 alfa0 alfa1 d-alfa d-r)
#;(parameterize ((attractors (list (+y (sph 10 0 angle-0) pos-0)
                                 (+y (sph 10 0 angle-1) pos-1)
                                 (+y (sph 10 0 angle-2) pos-2))))
  (map (lambda (p) (sphere p 0.5)) (attractors))
  (trelica-ondulada (xyz 0 0 0) 10 9 1.0 10 0 -pi/2 pi/2 0 4pi (/ pi 8) 0.1))