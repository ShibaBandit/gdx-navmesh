# gdx-navmesh

Contents:

- [Overview](#overview)
  - [Dependency](#dependency)
- [Examples](#examples)
- [Issues](#issues)
- [Contributing](#contributing)
- [License](#license)
- [Authors](#authors)

## Overview

LibGDX AI navigation mesh using constrained Delaunay triangulation.

Provides:

 * NavMeshPathFinder for high-level navmesh pathfinding.
 * NavMeshClipper for pre-processing JTS Geometry to poly2tri geometry.
 * NavMeshStringPuller for taking a pathing route and winding it around the navmesh.

See:

 * [Wiki - Navigation Mesh](https://en.wikipedia.org/wiki/Navigation_mesh)
 * [GitHub - libGDX](https://github.com/libgdx/libgdx/wiki) for libGDX wiki.
 * [GitHub - gdx-ai](https://github.com/libgdx/gdx-ai/wiki) for gdx-ai extension wiki.
 * [GitHub - JTS](https://github.com/locationtech/jts) for JTS source.

#### Dependency

###### Maven

```xml
<dependency>
    <groupId>io.github.shibabandit</groupId>
    <artifactId>gdx-navmesh</artifactId>
    <version>0.1.0</version>
</dependency>
```

###### Gradle

```
compile 'io.github.shibabandit:gdx-navmesh:0.1.0'
```

## Examples

See the src/test subdirectory for examples.

## Issues

#### Reporting

Use the Github issue tracker.

#### Known Issues

## Contributing

See [CONTRIBUTING](CONTRIBUTING.md)

## License

http://www.apache.org/licenses/LICENSE-2.0.html

## Authors

See [AUTHORS](AUTHORS.md)