# CoBot Clasificador de Colores — Documentación

Sitio de documentación académica del proyecto **CoBot Manipulador Clasificador de Colores con Visión Artificial y Cinemática Inversa**.

## Información del Proyecto

| Campo | Detalle |
|---|---|
| **Estudiante** | Elias Santiago Jiménez Hernández |
| **Materia** | Control Avanzado de Robots 2025 |
| **Institución** | Universidad Iberoamericana (IBERO) |
| **Robot** | Universal Robots UR3 — 6 GDL |

## Tecnologías del Sitio

- [Jekyll](https://jekyllrb.com/) — Generador de sitios estáticos
- [Just the Docs](https://github.com/just-the-docs/just-the-docs) — Tema de documentación
- [GitHub Pages](https://pages.github.com/) — Hosting gratuito

## Estructura de Archivos

```
├── _config.yml              # Configuración Jekyll
├── _includes/
│   └── footer_custom.html  # Footer personalizado
├── index.md                 # Portada (nav_order: 1)
├── 01-introduccion.md       # Hardware y objetivos
├── 02-cinematica-directa.md # Parámetros DH y matrices
├── 03-control-cinematico.md # Jacobiano y control proporcional
├── 04-cinematica-inversa.md # Página padre (has_children: true)
├── 04a-metodo-geometrico.md # IK analítica paso a paso
├── 04b-levenberg-marquardt.md # IK numérica LM
├── 04c-trayectorias.md      # Quínticos y trapezoidales
├── 05-implementacion-industrial.md
├── 06-vision-artificial.md  # Pipeline HSV + ArUco
├── 07-resultados.md         # Métricas y comparativas
├── 08-codigo-fuente.md      # Descripción de módulos
├── assets/img/              # Imágenes del proyecto
└── .github/workflows/
    └── deploy.yml           # GitHub Actions para Pages
```

## Despliegue en GitHub Pages

1. **Fork o clona** este repositorio
2. Ve a **Settings → Pages**
3. En *Source*, selecciona **GitHub Actions**
4. Haz push a `main` — la acción desplegará automáticamente

## Desarrollo Local

```bash
# Instalar dependencias
bundle install

# Servir localmente
bundle exec jekyll serve

# Abrir en navegador
open http://localhost:4000
```

## Licencia

Documentación académica. Todos los derechos reservados © 2025 Elias Santiago Jiménez Hernández — IBERO.
