"""Compatibility entry point for the route-cost and PUC node."""
from route_cost_puc_pynode.route_cost_puc_pynode import RouteCostPucNode, main

RouteCostPucEnergyNode = RouteCostPucNode

if __name__ == '__main__':
    main()
