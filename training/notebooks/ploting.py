import matplotlib.pyplot as plt
from matplotlib.patches import Patch

def graficar_eda_trial(df, subject_id, trial_key=None, sensores=[1, 2]):
    """
    Grafica la aceleración (ax) por fase para un sujeto y trial específicos.
    
    Parámetros:
    - df: DataFrame con los datos.
    - subject_id: ID del sujeto a filtrar.
    - trial_key: El ID del trial (si es None, toma el primero disponible).
    - sensores: Lista de IDs de sensores a graficar.
    """
    
    # 1. Filtrar por sujeto y obtener trials disponibles
    df_sujeto = df[df['subject_id'] == subject_id]
    trials_disponibles = df_sujeto['trial_key'].unique()
    
    if len(trials_disponibles) == 0:
        print(f"No se encontraron datos para el sujeto: {subject_id}")
        return

    # 2. Selección de trial
    if trial_key is None or trial_key not in trials_disponibles:
        trial_key = trials_disponibles[0]
        print(f"Trial no especificado o no encontrado. Usando: {trial_key}")
    
    trial_df = df_sujeto[df_sujeto['trial_key'] == trial_key].copy()
    
    # 3. Configuración de la figura
    fig, axes = plt.subplots(len(sensores), 1, figsize=(13, 3 * len(sensores)), sharex=False)
    # Asegurar que axes sea iterable aunque solo sea un sensor
    if len(sensores) == 1: axes = [axes]
    
    phase_colors = {'loading':'#2196F3', 'midstance':'#4CAF50',
                    'terminal':'#FF9800', 'swing':'#9C27B0'}

    # 4. Graficado
    for ax_idx, sid in enumerate(sensores):
        sub = trial_df[trial_df['sensor_id'] == sid].sort_values('timestamp_arduino_ms')
        
        if sub.empty:
            axes[ax_idx].set_title(f'Sensor {sid} — sin datos')
            continue
            
        t = sub['timestamp_arduino_ms'].values
        axes[ax_idx].plot(t, sub['ax'].values, lw=0.9, color='gray', label='ax')
        
        # Coloreado de fondo por fases
        prev_phase = None
        start_t = t[0]
        for i, (ti, ph) in enumerate(zip(t, sub['phase'].values)):
            if ph != prev_phase:
                if prev_phase is not None:
                    axes[ax_idx].axvspan(start_t, ti, alpha=0.18, 
                                         color=phase_colors.get(prev_phase, 'white'))
                start_t = ti
                prev_phase = ph
        axes[ax_idx].axvspan(start_t, t[-1], alpha=0.18, color=phase_colors.get(prev_phase, 'white'))
        
        axes[ax_idx].set_title(f'Sensor {sid} — ax — trial {trial_key}')
        axes[ax_idx].set_ylabel('Aceleración (g)')
        axes[ax_idx].set_xlabel('timestamp_arduino_ms')

    # 5. Leyenda y ajustes finales
    handles = [Patch(color=c, alpha=0.5, label=p) for p, c in phase_colors.items()]
    fig.legend(handles=handles, loc='upper right', ncol=4, fontsize=9)
    plt.tight_layout()
    plt.show()

# --- Ejemplo de uso ---
# graficar_eda_trial(df, 'AHC', 'AHC_t004', sensores=[1, 2])