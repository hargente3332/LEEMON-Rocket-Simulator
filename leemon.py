#!/usr/bin/env python3
"""
LEEMON V2 - Portable Distribution
Rocket Flight Simulator
"""

import sys
from pathlib import Path

LEEMON_ROOT = Path(__file__).parent.resolve()

__PYTHON_FOLDER__ = LEEMON_ROOT / "python"
if __PYTHON_FOLDER__.exists():
    sys.path.insert(0, str(__PYTHON_FOLDER__))
else:
    raise ImportError(f"Python folder not found at {__PYTHON_FOLDER__}")

try:
    from leemonSim import LEEMONSimulator as _LEEMONSimulator, randomizeParam
    from analysisTools import FlightAnalyzer as _FlightAnalyzer, VariabilityAnalyzer as _VariabilityAnalyzer
except ImportError as e:
    raise ImportError(f"Failed to load LEEMON modules: {e}")

__version__ = "2.0.0"


def resolve_path(relative_path: str) -> str:
    """
    Convierte una ruta relativa a una ruta absoluta basada en LEEMON_ROOT.
    
    Args:
        relative_path: Ruta relativa desde la raíz de LEEMON (ej: 'rockets/viper/results/viperData.csv')
    
    Returns:
        Ruta absoluta como string
    """
    return str(LEEMON_ROOT / relative_path)


class LEEMONSimulator(_LEEMONSimulator):
    """Wrapper de LEEMONSimulator que usa LEEMON_ROOT automáticamente"""
    
    def __init__(self, project_root=None):
        if project_root is None:
            project_root = LEEMON_ROOT
        super().__init__(project_root=project_root)


class FlightAnalyzer(_FlightAnalyzer):
    """Wrapper de FlightAnalyzer que resuelve rutas automáticamente"""
    
    def __init__(self, csv_path: str):
        # Si la ruta no es absoluta, resolverla desde LEEMON_ROOT
        path = Path(csv_path)
        if not path.is_absolute():
            csv_path = str(LEEMON_ROOT / csv_path)
        super().__init__(csv_path)


class VariabilityAnalyzer(_VariabilityAnalyzer):
    """Wrapper de VariabilityAnalyzer que resuelve rutas automáticamente"""
    
    def __init__(self, results_folder: str):
        # Si la ruta no es absoluta, resolverla desde LEEMON_ROOT
        path = Path(results_folder)
        if not path.is_absolute():
            results_folder = str(LEEMON_ROOT / results_folder)
        super().__init__(results_folder)


__all__ = ['LEEMONSimulator', 'FlightAnalyzer', 'VariabilityAnalyzer', 'randomizeParam', 'LEEMON_ROOT', 'resolve_path']

if __name__ == "__main__":
    print(f"LEEMON V2 Portable - Version {__version__}")
    print(f"Root: {LEEMON_ROOT}")
    print("\nUsage: python rockets/myRocket/mySimulation.py")
