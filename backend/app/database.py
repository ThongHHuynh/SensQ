from sqlalchemy.ext.asyncio import async_sessionmaker, create_async_engine

from .config import DATABASE_URL
from .models import Base, RobotEvent, RobotSnapshot


engine = create_async_engine(DATABASE_URL, pool_pre_ping=True)
AsyncSessionLocal = async_sessionmaker(engine, expire_on_commit=False)


async def init_db() -> None:
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


async def save_event(source: str, message: str, level: str = "info") -> None:
    async with AsyncSessionLocal() as session:
        session.add(RobotEvent(source=source, message=message, level=level))
        await session.commit()


async def save_snapshot(payload: dict) -> None:
    async with AsyncSessionLocal() as session:
        session.add(RobotSnapshot(payload=payload))
        await session.commit()
