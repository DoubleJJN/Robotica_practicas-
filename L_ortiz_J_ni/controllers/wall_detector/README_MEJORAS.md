# Controlador Wall Detector - Comportamiento Reactivo

## Resumen del Sistema Implementado

He simplificado tu controlador del robot Pioneer 2 para crear un **sistema completamente reactivo de seguimiento de paredes** usando únicamente la cámara frontal. Sin puntos de referencia, sin navegación dirigida, solo comportamiento reactivo puro.

## Funcionalidades del Sistema Reactivo

### 1. **Detección de Paredes Optimizada**
- **Detección específica** para paredes blancas/grisáceas de tu entorno
- **Calibrada para iluminación**: Threshold 90 para paredes que se ven grises
- **Tres regiones**: Izquierda (15%), Centro (70%), Derecha (15%)
- **Criterio 70%**: Mantiene tu configuración original de cobertura

### 2. **Comportamiento Completamente Reactivo**
- **Sin objetivos**: No necesita puntos de destino
- **Sin odometría**: No rastrea posición o orientación
- **Reacción inmediata**: Responde solo a lo que ve la cámara
- **Exploración continua**: Se mueve por el mundo siguiendo paredes

### 3. **Estados Reactivos Simples**
- `FORWARD`: Avanzar cuando no hay obstáculos
- `WALL_FOLLOW_RIGHT`: Seguir pared del lado derecho
- `WALL_FOLLOW_LEFT`: Seguir pared del lado izquierdo  
- `TURN_AROUND`: Dar vuelta cuando está completamente bloqueado
- `EXPLORING`: Exploración básica buscando paredes

### 4. **Comportamientos Reactivos**
- **Avance libre**: Se mueve hacia adelante cuando no detecta paredes
- **Seguimiento de paredes**: Mantiene una pared a su lado mientras avanza
- **Cambio de lado**: Cambia de seguir pared derecha a izquierda según obstáculos
- **Giro completo**: Da media vuelta cuando está completamente bloqueado

## Código Mejorado

### Estructura Principal (`MyRobot.h`)
```cpp
// Estados reactivos simples para seguimiento de paredes
enum ReactiveState {
    FORWARD,           // Avanzar sin obstáculos
    WALL_FOLLOW_RIGHT, // Seguir pared del lado derecho
    WALL_FOLLOW_LEFT,  // Seguir pared del lado izquierdo
    TURN_AROUND,       // Dar vuelta cuando está bloqueado
    EXPLORING          // Exploración básica
};
```

### Métodos Reactivos Implementados (`MyRobot.cpp`)

1. **`detectWall()`**: Detección simple y eficiente de paredes blancas/grisáceas
2. **`moveForward()`**: Avance recto cuando no hay obstáculos
3. **`followRightWall()`**: Seguimiento de pared del lado derecho
4. **`followLeftWall()`**: Seguimiento de pared del lado izquierdo
5. **`turnAround()`**: Giro completo cuando está bloqueado
6. **`explore()`**: Comportamiento de exploración básica

## Cómo Usar el Sistema Reactivo

### Uso Simple - Sin Configuración Necesaria
```cpp
MyRobot* robot = new MyRobot();
robot->run();  // ¡Eso es todo! El robot explora reactivamente
```

### Comportamiento Automático
- **No necesita objetivos**: El robot explora por sí solo
- **No requiere configuración**: Funciona inmediatamente  
- **Adaptación automática**: Cambia comportamiento según lo que ve
- **Exploración continua**: Recorre el mundo siguiendo paredes

## Algoritmos Reactivos Implementados

### 1. **Detección Simple de Paredes**
```cpp
bool wall_detected = (white_percentage > 70%);  // Calibrado para tu entorno
// Solo analiza píxeles blancos/grisáceos con threshold 90
```

### 2. **Seguimiento de Pared Derecha**
```cpp
_left_speed = 5.0;   // Rueda izquierda más rápida
_right_speed = 3.0;  // Rueda derecha más lenta = curva a la izquierda
```

### 3. **Seguimiento de Pared Izquierda**
```cpp
_left_speed = 3.0;   // Rueda izquierda más lenta
_right_speed = 5.0;  // Rueda derecha más rápida = curva a la derecha
```

### 4. **Máquina de Estados Reactiva**
```cpp
switch(_current_state) {
    case FORWARD: /* avanzar si no hay paredes */
    case WALL_FOLLOW_RIGHT: /* seguir pared derecha */  
    case WALL_FOLLOW_LEFT: /* seguir pared izquierda */
    case TURN_AROUND: /* dar vuelta si bloqueado */
}
```

## Ventajas del Sistema Reactivo

1. **Solo Visión**: No depende de sensores adicionales  
2. **Completamente Reactivo**: No necesita puntos de referencia ni objetivos
3. **Detección Optimizada**: Calibrada específicamente para paredes blancas/grisáceas
4. **Exploración Autónoma**: Recorre el mundo automáticamente
5. **Simplicidad**: Código más limpio y fácil de entender
6. **Robustez**: Comportamiento estable sin odometría compleja

## Parámetros Configurables

En `MyRobot.h` puedes ajustar:
```cpp
#define THRESHOLD_WHITE 90        // Threshold para paredes blancas/grisáceas (calibrado para iluminación)
#define MIN_OBSTACLE_PERCENTAGE 70 // % mínimo para considerar pared (como en tu código original)
```

## Ejemplo de Uso Completo

El archivo `wall_detector.cpp` muestra un ejemplo completo:
```cpp
int main() {
    MyRobot* robot = new MyRobot();
    robot->run();    // ¡Solo esto! Exploración reactiva automática
    delete robot;
    return 0;
}
```

## Posibles Extensiones Futuras

1. **Memoria de caminos**: Recordar rutas ya exploradas
2. **Variaciones de seguimiento**: Alternar entre pared derecha e izquierda
3. **Detección de espacios abiertos**: Cambiar comportamiento en áreas grandes
4. **Optimización de velocidades**: Ajustar velocidades según tipo de pared
5. **Comportamiento emergencia**: Reacciones especiales para situaciones atípicas

## Compilación

Para compilar en Webots:
1. Abre el proyecto en Webots
2. Selecciona el controlador `wall_detector`
3. Compila automáticamente al ejecutar la simulación

El robot ahora **explora reactivamente todo el mundo** siguiendo paredes y adaptándose automáticamente a los obstáculos usando únicamente la información visual de la cámara frontal. ¡Sin configuración, sin objetivos, puro comportamiento reactivo!
