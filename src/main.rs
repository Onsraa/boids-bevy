mod globals;
mod resources;
mod ui;

use crate::globals::*;
use crate::ui::UiPlugin;
use bevy::prelude::*;
use rand::prelude::*;
use crate::resources::grid::GridSettings;

#[derive(Component, Default)]
struct Velocity(Vec2);

#[derive(Component)]
#[require(Transform, Velocity)]
struct Boid {
    mass: f32,
    max_speed: f32,
    max_force: f32,
}

impl Default for Boid {
    fn default() -> Self {
        Self {
            mass: BOID_MASS,
            max_speed: BOID_MAX_SPEED,
            max_force: BOID_MAX_FORCE,
        }
    }
}

#[derive(Resource)]
struct BoidSettings {
    separation_radius: f32,
    alignment_radius: f32,
    cohesion_radius: f32,
    separation_weight: f32,
    alignment_weight: f32,
    cohesion_weight: f32,
    view_angle: f32,
}

impl Default for BoidSettings {
    fn default() -> Self {
        Self {
            separation_radius: 25.0,
            alignment_radius: 50.0,
            cohesion_radius: 50.0,
            separation_weight: 1.5,
            alignment_weight: 1.0,
            cohesion_weight: 1.0,
            view_angle: 4.7,
        }
    }
}

// Fonction utilitaire pour vérifier si un boid est dans le champ de vision
fn is_in_view(boid_pos: Vec2, boid_dir: Vec2, other_pos: Vec2, view_angle: f32) -> bool {
    let to_other = (other_pos - boid_pos).normalize_or_zero();
    let angle = boid_dir.dot(to_other).acos();
    angle <= view_angle / 2.0
}

// Structure pour stocker les voisins d'un boid
struct Neighbors {
    separation: Vec<(Vec2, Vec2)>, // (position, vélocité)
    alignment: Vec<(Vec2, Vec2)>,
    cohesion: Vec<(Vec2, Vec2)>,
}

fn calculate_separation(boid_pos: Vec2, neighbors: &[(Vec2, Vec2)]) -> Vec2 {
    if neighbors.is_empty() {
        return Vec2::ZERO;
    }

    // fold() est comme reduce() : accumule les valeurs
    // Ici, on additionne tous les vecteurs de répulsion
    let steer = neighbors
        .iter()
        .fold(Vec2::ZERO, |acc, &(neighbor_pos, _)| {
            let diff = boid_pos - neighbor_pos;
            let distance = diff.length();
            // Plus le voisin est proche, plus la répulsion est forte
            if distance > 0.0 {
                acc + diff.normalize() / distance
            } else {
                acc
            }
        });

    steer.normalize_or_zero()
}

fn calculate_alignment(boid_velocity: Vec2, neighbors: &[(Vec2, Vec2)]) -> Vec2 {
    if neighbors.is_empty() {
        return Vec2::ZERO;
    }

    // Calcul de la vélocité moyenne des voisins
    let avg_velocity = neighbors
        .iter()
        .map(|&(_, vel)| vel) // map() transforme chaque élément
        .sum::<Vec2>()
        / neighbors.len() as f32;

    (avg_velocity - boid_velocity).normalize_or_zero()
}

fn calculate_cohesion(boid_pos: Vec2, neighbors: &[(Vec2, Vec2)]) -> Vec2 {
    if neighbors.is_empty() {
        return Vec2::ZERO;
    }

    // Centre de masse du groupe
    let center = neighbors.iter().map(|&(pos, _)| pos).sum::<Vec2>() / neighbors.len() as f32;

    (center - boid_pos).normalize_or_zero()
}

fn boid_movement_system(
    mut boids: Query<(Entity, &mut Transform, &mut Velocity, &Boid)>,
    settings: Res<BoidSettings>,
    time: Res<Time>,
) {
    // Collecter toutes les positions pour éviter les problèmes de borrowing
    let boid_data: Vec<_> = boids
        .iter()
        .map(|(entity, transform, velocity, _)| (entity, transform.translation.xy(), velocity.0))
        .collect();

    for (entity, mut transform, mut velocity, boid) in boids.iter_mut() {
        let pos = transform.translation.xy();
        let vel = velocity.0;
        let direction = vel.normalize_or_zero();

        // Trouver les voisins dans chaque rayon
        let mut neighbors = Neighbors {
            separation: Vec::new(),
            alignment: Vec::new(),
            cohesion: Vec::new(),
        };

        // iter() crée un itérateur sur les éléments
        // filter() garde seulement les éléments qui passent le test
        for &(other_entity, other_pos, other_vel) in boid_data.iter() {
            if entity == other_entity {
                continue;
            }

            let distance = pos.distance(other_pos);
            if !is_in_view(pos, direction, other_pos, settings.view_angle) {
                continue;
            }

            if distance < settings.separation_radius {
                neighbors.separation.push((other_pos, other_vel));
            }
            if distance < settings.alignment_radius {
                neighbors.alignment.push((other_pos, other_vel));
            }
            if distance < settings.cohesion_radius {
                neighbors.cohesion.push((other_pos, other_vel));
            }
        }

        // Calculer les trois forces
        let separation =
            calculate_separation(pos, &neighbors.separation) * settings.separation_weight;
        let alignment = calculate_alignment(vel, &neighbors.alignment) * settings.alignment_weight;
        let cohesion = calculate_cohesion(pos, &neighbors.cohesion) * settings.cohesion_weight;

        // Combiner les forces
        let desired = (separation + alignment + cohesion).normalize_or_zero() * boid.max_speed;
        let steer = (desired - vel).clamp_length_max(boid.max_force);

        // Appliquer la physique
        let acceleration = steer / boid.mass;
        velocity.0 += acceleration * time.delta_secs();
        velocity.0 = velocity.0.clamp_length_max(boid.max_speed);

        // Mettre à jour position et rotation
        transform.translation += velocity.0.extend(0.0) * time.delta_secs();

        // Faire pointer le triangle dans la direction du mouvement
        if velocity.0.length() > 0.0 {
            let angle = velocity.0.y.atan2(velocity.0.x) - std::f32::consts::FRAC_PI_2;
            transform.rotation = Quat::from_rotation_z(angle);
        }
    }
}

fn wrap_around_edges(mut boids: Query<&mut Transform, With<Boid>>, grid_settings: Res<GridSettings>) {
    for mut transform in boids.iter_mut() {
        let mut pos = transform.translation;

        // Téléportation de l'autre côté si on sort des limites
        if pos.x > grid_settings.width / 2.0 {
            pos.x = -grid_settings.width / 2.0;
        }
        if pos.x < -grid_settings.width / 2.0 {
            pos.x = grid_settings.width / 2.0;
        }
        if pos.y > grid_settings.height / 2.0 {
            pos.y = -grid_settings.height / 2.0;
        }
        if pos.y < -grid_settings.height / 2.0 {
            pos.y = grid_settings.height / 2.0;
        }

        transform.translation = pos;
    }
}

fn generate_boids(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    grid_settings: Res<GridSettings>,
) {
    let mut rng = rand::rng();

    let mesh = meshes.add(Triangle2d::new(
        Vec2::Y * BOID_SIZE * 1.6,
        Vec2::new(-BOID_SIZE, -BOID_SIZE),
        Vec2::new(BOID_SIZE, -BOID_SIZE),
    ));

    let color = Color::WHITE;

    for _ in 0..NUMBER_BOIDS {
        let angle = rng.random::<f32>() * std::f32::consts::TAU;
        let initial_velocity = Vec2::new(angle.cos(), angle.sin()) * 50.0;

        commands.spawn((
            Boid::default(),
            Transform::from_xyz(
                rng.random_range(-grid_settings.width / 2.0..grid_settings.width / 2.0),
                rng.random_range(-grid_settings.height / 2.0..grid_settings.height / 2.0),
                0.0,
            ),
            Velocity(initial_velocity),
            Mesh2d(mesh.clone()),
            MeshMaterial2d(materials.add(color)),
        ));
    }
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d::default());
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(UiPlugin)
        .init_resource::<GridSettings>()
        .init_resource::<BoidSettings>()
        .add_systems(Startup, (setup, generate_boids))
        .add_systems(Update, (boid_movement_system, wrap_around_edges).chain())
        .run();
}
